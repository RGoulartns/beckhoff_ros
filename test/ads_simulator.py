#!/usr/bin/env python3
"""
Minimal TwinCAT 3 PLC simulator speaking ADS over TCP, to test beckhoff_ros without a PLC.

It serves the variables described in a beckhoff_ros configuration file. Their memory layout is
computed with ctypes, exactly like the Python version of beckhoff_ros did, and values are kept
as raw bytes. Supported ADS services: read device info, read state, read/write/read-write by
symbol handle or name, and device notifications (sent when a value changes).

Usage: ads_simulator.py config/params.json [--host 127.0.0.1] [--value GVL.var1=True ...]
The AMS Net Id of the PLC must contain the address it listens on (e.g. 127.0.0.1.1.1).
"""

import argparse
import ast
import ctypes
import json
import socket
import socketserver
import struct
import threading
import time

ADS_TCP_PORT = 48898

READ_DEVICE_INFO = 1
READ = 2
WRITE = 3
READ_STATE = 4
WRITE_CONTROL = 5
ADD_NOTIFICATION = 6
DEL_NOTIFICATION = 7
NOTIFICATION = 8
READ_WRITE = 9

SYM_HNDBYNAME = 0xF003
SYM_VALBYNAME = 0xF004
SYM_VALBYHND = 0xF005
SYM_RELEASEHND = 0xF006

ERR_SRVNOTSUPP = 0x701
ERR_INVALIDGRP = 0x702
ERR_INVALIDSIZE = 0x705
ERR_SYMBOLNOTFOUND = 0x710
ERR_NOTIFYHNDINVALID = 0x714

AMS_HEADER = struct.Struct('<6sH6sHHHIII')

CTYPES = {
    'bool': ctypes.c_bool, 'int': ctypes.c_int16, 'int16': ctypes.c_int16,
    'uint16': ctypes.c_uint16, 'uint8': ctypes.c_uint8, 'real': ctypes.c_float,
    'int8': ctypes.c_int8, 'int32': ctypes.c_int32, 'uint32': ctypes.c_uint32,
    'int64': ctypes.c_int64, 'uint64': ctypes.c_uint64, 'float': ctypes.c_float,
    'double': ctypes.c_double, 'sint': ctypes.c_int8, 'usint': ctypes.c_uint8,
    'byte': ctypes.c_uint8, 'uint': ctypes.c_uint16, 'word': ctypes.c_uint16,
    'dint': ctypes.c_int32, 'udint': ctypes.c_uint32, 'dword': ctypes.c_uint32,
    'lint': ctypes.c_int64, 'ulint': ctypes.c_uint64, 'lword': ctypes.c_uint64,
    'lreal': ctypes.c_double,
}


def build_ctypes(plc_config):
    """Return {variable name: ctypes type}, built like the Python version of beckhoff_ros."""
    types = dict(CTYPES)

    def resolve(spec):
        if isinstance(spec, list):
            return types[spec[0]] * spec[1]
        return types[spec]

    pack = plc_config.get('pack', 0)
    for name, fields in plc_config['structures'].items():
        types[name] = type(name, (ctypes.Structure,), {
            '_pack_': pack,
            '_fields_': [(field, resolve(spec)) for field, spec in fields.items()]})
    return {name: resolve(spec) for name, spec in plc_config['variables'].items()}


def to_ctype(ctype, value):
    """Convert a Python value (dict for structures, list for arrays) to a ctypes instance."""
    if isinstance(value, (list, tuple)):
        return ctype(*[to_ctype(ctype._type_, item) for item in value])
    if isinstance(value, dict):
        instance = ctype()
        for field, field_type in ctype._fields_:
            setattr(instance, field, to_ctype(field_type, value[field]))
        return instance
    return ctype(value)


def to_python(value):
    """Convert a ctypes instance (or the Python value of a simple ctypes field) to Python."""
    if isinstance(value, ctypes.Array):
        return [to_python(item) for item in value]
    if isinstance(value, ctypes.Structure):
        return {field: to_python(getattr(value, field)) for field, _ in value._fields_}
    if isinstance(value, ctypes._SimpleCData):
        return value.value
    return value


class Variable:
    """A PLC variable: its type and current raw value."""

    def __init__(self, name, ctype):
        self.name = name
        self.ctype = ctype
        self.data = bytes(ctypes.sizeof(ctype))


class AdsSimulator:
    """ADS server holding the variables of a beckhoff_ros configuration."""

    def __init__(self, plc_config, host=None, port=ADS_TCP_PORT, values=None):
        net_id = plc_config['net_id']
        self.host = host or '.'.join(net_id.split('.')[:4])
        self.port = port
        self.variables = {
            name.lower(): Variable(name, ctype)
            for name, ctype in build_ctypes(plc_config).items()}
        self.lock = threading.RLock()
        self.connections = set()
        self.handles = {}          # symbol handle -> Variable
        self.notifications = {}    # notification handle -> (connection, Variable, header)
        self.next_handle = 0x1000
        self.next_notification = 1
        self.writes = []           # (variable name, bytes) written by clients
        for name, value in (values or {}).items():
            self.set_value(name, value)
        self.server = None
        self.thread = None

    # Access from tests / PLC logic

    def variable(self, name):
        """Return the Variable called name, ignoring case like TwinCAT does."""
        return self.variables[name.lower()]

    def set_value(self, name, value):
        """Change a variable, as the PLC program would, notifying the clients."""
        variable = self.variable(name)
        self.write(variable, bytes(to_ctype(variable.ctype, value)))

    def get_value(self, name):
        """Return the value of a variable as a Python object."""
        variable = self.variable(name)
        return to_python(variable.ctype.from_buffer_copy(variable.data))

    def write(self, variable, data, client=False):
        """Store a new value and send it to the notifications registered for it."""
        with self.lock:
            if client:
                self.writes.append((variable.name, bytes(data)))
            changed = data != variable.data
            variable.data = bytes(data)
            if changed:
                for handle, (connection, target, header) in list(self.notifications.items()):
                    if target is variable:
                        connection.send_notification(header, handle, variable.data)

    # Server

    def start(self):
        """Start serving in a background thread."""
        simulator = self

        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        class Handler(AmsConnection):
            def setup(self):
                self.simulator = simulator
                super().setup()

        self.server = Server((self.host, self.port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        """Stop serving and close the connections."""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            with self.lock:
                for connection in list(self.connections):
                    connection.close()
                self.notifications.clear()

    # ADS services

    def handle(self, connection, command, data, header):
        """Process a request, return (ADS result, response data)."""
        if command == READ_DEVICE_INFO:
            return 0, struct.pack('<BBH16s', 3, 1, 4024, b'Plc30 App')
        if command == READ_STATE:
            return 0, struct.pack('<HH', 5, 0)  # ADSSTATE_RUN
        if command == WRITE_CONTROL:
            return 0, b''
        if command == READ:
            group, offset, length = struct.unpack_from('<III', data)
            error, value = self.read(group, offset, length)
            return error, struct.pack('<I', len(value)) + value
        if command == WRITE:
            group, offset, length = struct.unpack_from('<III', data)
            return self.write_request(group, offset, data[12:12 + length]), b''
        if command == READ_WRITE:
            group, offset, read_length, write_length = struct.unpack_from('<IIII', data)
            error, value = self.read_write(group, offset, read_length, data[16:16 + write_length])
            return error, struct.pack('<I', len(value)) + value
        if command == ADD_NOTIFICATION:
            group, offset, length, mode, max_delay, cycle_time = struct.unpack_from('<6I', data)
            return self.add_notification(connection, group, offset, length, header)
        if command == DEL_NOTIFICATION:
            handle, = struct.unpack_from('<I', data)
            with self.lock:
                found = self.notifications.pop(handle, None)
            return (0 if found else ERR_NOTIFYHNDINVALID), b''
        return ERR_SRVNOTSUPP, b''

    def by_handle(self, group, offset):
        if group != SYM_VALBYHND:
            return None
        return self.handles.get(offset)

    def by_name(self, name):
        return self.variables.get(name.rstrip(b'\0').decode().lower())

    def read(self, group, offset, length):
        variable = self.by_handle(group, offset)
        if variable is None:
            return (ERR_SYMBOLNOTFOUND if group == SYM_VALBYHND else ERR_INVALIDGRP), b''
        if length != len(variable.data):
            return ERR_INVALIDSIZE, b''
        return 0, variable.data

    def write_request(self, group, offset, value):
        if group == SYM_RELEASEHND:
            handle, = struct.unpack('<I', value)
            with self.lock:
                self.handles.pop(handle, None)
            return 0
        variable = self.by_handle(group, offset)
        if variable is None:
            return ERR_SYMBOLNOTFOUND if group == SYM_VALBYHND else ERR_INVALIDGRP
        if len(value) != len(variable.data):
            return ERR_INVALIDSIZE
        self.write(variable, value, client=True)
        return 0

    def read_write(self, group, offset, read_length, value):
        if group in (SYM_HNDBYNAME, SYM_VALBYNAME):
            variable = self.by_name(value)
            if variable is None:
                return ERR_SYMBOLNOTFOUND, b''
            if group == SYM_VALBYNAME:
                return 0, variable.data[:read_length]
            with self.lock:
                handle = self.next_handle
                self.next_handle += 1
                self.handles[handle] = variable
            return 0, struct.pack('<I', handle)
        return ERR_INVALIDGRP, b''

    def add_notification(self, connection, group, offset, length, header):
        variable = self.by_handle(group, offset)
        if variable is None:
            return ERR_SYMBOLNOTFOUND, b''
        if length != len(variable.data):
            return ERR_INVALIDSIZE, b''
        with self.lock:
            handle = self.next_notification
            self.next_notification += 1
            self.notifications[handle] = (connection, variable, header)

        def send_current_value():  # like TwinCAT, shortly after the notification is added
            time.sleep(0.01)
            with self.lock:
                if handle in self.notifications:
                    connection.send_notification(header, handle, variable.data)

        threading.Thread(target=send_current_value, daemon=True).start()
        return 0, struct.pack('<I', handle)


class AmsConnection(socketserver.BaseRequestHandler):
    """A client connection: AMS/TCP frames in, responses and notifications out."""

    def setup(self):
        self.send_lock = threading.Lock()
        self.closed = False
        with self.simulator.lock:
            self.simulator.connections.add(self)

    def receive(self, size):
        data = b''
        while len(data) < size:
            chunk = self.request.recv(size - len(data))
            if not chunk:
                raise ConnectionError('connection closed')
            data += chunk
        return data

    def send(self, frame):
        with self.send_lock:
            if not self.closed:
                try:
                    self.request.sendall(struct.pack('<HI', 0, len(frame)) + frame)
                except OSError:
                    self.closed = True

    def close(self):
        self.closed = True
        try:
            self.request.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def handle(self):
        try:
            while True:
                _, length = struct.unpack('<HI', self.receive(6))
                frame = self.receive(length)
                (target_net_id, target_port, source_net_id, source_port, command, flags, _,
                 _, invoke_id) = AMS_HEADER.unpack_from(frame)
                if flags & 0x0001:  # a response
                    continue
                header = (source_net_id, source_port, target_net_id, target_port)
                error, data = self.simulator.handle(
                    self, command, frame[AMS_HEADER.size:], header)
                payload = struct.pack('<I', error) + (data if error == 0 else b'')
                if command in (READ, READ_WRITE) and error != 0:
                    payload += struct.pack('<I', 0)
                self.send(AMS_HEADER.pack(
                    source_net_id, source_port, target_net_id, target_port, command, 0x0005,
                    len(payload), 0, invoke_id) + payload)
        except (ConnectionError, OSError):
            pass
        finally:
            self.closed = True
            with self.simulator.lock:
                self.simulator.connections.discard(self)
                for handle, (connection, _, _) in list(self.simulator.notifications.items()):
                    if connection is self:
                        del self.simulator.notifications[handle]

    def send_notification(self, header, handle, data):
        client_net_id, client_port, plc_net_id, plc_port = header
        # FILETIME: 100 ns intervals since 1601-01-01
        timestamp = int(time.time() * 1e7) + 116444736000000000
        # 1 stamp: timestamp, 1 sample: notification handle, size, data
        stamps = struct.pack('<IQIII', 1, timestamp, 1, handle, len(data)) + data
        payload = struct.pack('<I', len(stamps)) + stamps
        self.send(AMS_HEADER.pack(
            client_net_id, client_port, plc_net_id, plc_port, NOTIFICATION, 0x0004,
            len(payload), 0, 0) + payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('config', help='beckhoff_ros JSON configuration file')
    parser.add_argument('--host', help='address to listen on (default: from net_id)')
    parser.add_argument(
        '--value', action='append', default=[], metavar='NAME=VALUE',
        help='initial value of a variable, as a Python literal')
    args = parser.parse_args()
    with open(args.config) as config_file:
        plc_config = json.load(config_file)['PLC']
    values = {}
    for item in args.value:
        name, value = item.split('=', 1)
        values[name] = ast.literal_eval(value)
    simulator = AdsSimulator(plc_config, host=args.host, values=values).start()
    print('ADS simulator listening on %s:%d' % (simulator.host, simulator.port), flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        simulator.stop()


if __name__ == '__main__':
    main()
