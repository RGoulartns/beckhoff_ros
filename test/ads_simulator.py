#!/usr/bin/env python3
"""
Minimal TwinCAT 3 PLC simulator speaking ADS over TCP, to test beckhoff_ros without a PLC.

It serves the variables described in a beckhoff_ros configuration file. Their memory layout is
computed with ctypes and values are kept as raw bytes. Supported ADS services: read device
info, read state, read/write/read-write by symbol handle or name, symbol information and
version, sum commands, and device notifications (on change or cyclic).

With --cycle-ms, requests are processed at the end of the current PLC cycle, like TwinCAT does,
so the latency of the requests can be measured realistically (without it, they are processed
as soon as they are received).

Usage: ads_simulator.py config/params.json [--host 127.0.0.1] [--port 48898] [--cycle-ms 1]
                        [--value GVL.var1=True ...] [--control]
The AMS Net Id of the PLC must contain the address it listens on (e.g. 127.0.0.1.1.1).
With --control, commands are read from the standard input (see control()).
"""

import argparse
import ast
import collections
import ctypes
import json
import queue
import socket
import socketserver
import struct
import sys
import threading
import time

ADS_TCP_PORT = 48898
PLC_PORT = 851

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
SYM_VERSION = 0xF008
SYM_INFOBYNAMEEX = 0xF009
SUMUP_READ = 0xF080
SUMUP_WRITE = 0xF081
SUMUP_READWRITE = 0xF082

TRANS_SERVERCYCLE = 3
TRANS_SERVERONCHA = 4

ERR_TARGETPORTNOTFOUND = 0x6
ERR_SRVNOTSUPP = 0x701
ERR_INVALIDGRP = 0x702
ERR_INVALIDSIZE = 0x705
ERR_INVALIDPARM = 0x70B
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
    for name, fields in plc_config.get('structures', {}).items():
        types[name] = type(name, (ctypes.Structure,), {
            '_pack_': pack,
            '_fields_': [(field, resolve(spec)) for field, spec in fields.items()]})
    return {name: resolve(spec) for name, spec in plc_config['variables'].items()}


def plc_type_name(spec):
    """Name of a type of the configuration as TwinCAT shows it."""
    if isinstance(spec, list):
        return 'ARRAY [0..%d] OF %s' % (spec[1] - 1, plc_type_name(spec[0]))
    return spec.upper() if spec in CTYPES else spec


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


def filetime():
    """Return the current time as a Windows FILETIME (100 ns intervals since 1601-01-01)."""
    return int(time.time() * 1e7) + 116444736000000000


class Variable:
    """A PLC variable: its type and current raw value."""

    def __init__(self, name, ctype, type_name):
        self.name = name
        self.ctype = ctype
        self.type_name = type_name
        self.data = bytes(ctypes.sizeof(ctype))


class Notification:
    """A device notification added by a client."""

    def __init__(self, connection, variable, header, mode, cycle_time):
        self.connection = connection
        self.variable = variable
        self.header = header          # (client net id, client port, PLC net id, PLC port)
        self.mode = mode
        self.cycle = max(cycle_time, 1) * 1e-7  # seconds
        self.start = 0.0              # nothing is sent before (the response is sent first)
        self.next = 0.0               # next cyclic notification
        self.pending = True           # value to send (the current value is sent first)


class AdsSimulator:
    """ADS server holding the variables of a beckhoff_ros configuration."""

    def __init__(self, plc_config, host=None, port=ADS_TCP_PORT, values=None, cycle_ms=0.0):
        net_id = plc_config['net_id']
        self.host = host or '.'.join(net_id.split('.')[:4])
        self.port = port
        self.cycle = cycle_ms / 1000.0
        self.variables = {}
        types = build_ctypes(plc_config)
        for name, spec in plc_config['variables'].items():
            self.variables[name.lower()] = Variable(name, types[name], plc_type_name(spec))
        self.lock = threading.RLock()
        self.connections = set()
        self.handles = {}          # symbol handle -> Variable
        self.notifications = {}    # notification handle -> Notification
        self.next_handle = 0x1000
        self.next_notification = 1
        self.symbol_version = 1
        self.running = True        # False: the PLC runtime is stopped (port 851 not found)
        self.sum_commands = True   # False: like TwinCAT 2.10, without sum commands
        self.writes = []           # (variable name, bytes) written by clients
        self.requests = collections.Counter()  # (command, index group) -> count
        for name, value in (values or {}).items():
            self.set_value(name, value)
        self.server = None
        self.thread = None
        self.ticker = None
        self.stopping = threading.Event()

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
        """Store a new value and notify the notifications registered for it."""
        with self.lock:
            if client:
                self.writes.append((variable.name, bytes(data)))
            changed = data != variable.data
            variable.data = bytes(data)
            if changed:
                for notification in self.notifications.values():
                    if notification.variable is variable:
                        notification.pending = True
                if not self.cycle:
                    self.send_notifications()

    def change_program(self):
        """Simulate the download of a new PLC program: handles and notifications are lost."""
        with self.lock:
            self.handles.clear()
            self.notifications.clear()
            self.symbol_version = (self.symbol_version + 1) % 256

    def stop_runtime(self):
        """Simulate a stopped PLC runtime (TwinCAT in config mode): port 851 is not found."""
        with self.lock:
            self.running = False
            self.handles.clear()
            self.notifications.clear()

    def start_runtime(self):
        with self.lock:
            self.running = True

    def drop_connections(self):
        """Close the connections of the clients, like a PLC reboot or a network failure."""
        with self.lock:
            connections = list(self.connections)
            self.handles.clear()
            self.notifications.clear()
        for connection in connections:
            connection.close()

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

        self.stopping.clear()
        self.server = Server((self.host, self.port), Handler)
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.ticker = threading.Thread(target=self.tick, daemon=True)
        self.ticker.start()
        return self

    def stop(self):
        """Stop serving and close the connections."""
        self.stopping.set()
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
            self.drop_connections()

    def next_cycle(self, after):
        """Return the end of the PLC cycle in progress at the time `after`."""
        if not self.cycle:
            return after
        return (int(after / self.cycle) + 1) * self.cycle

    def tick(self):
        """PLC cycles: send the notifications (on change, and cyclic ones)."""
        period = self.cycle or 0.001
        while not self.stopping.is_set():
            now = time.monotonic()
            time.sleep(max(0.0, (int(now / period) + 1) * period - now))
            with self.lock:
                self.send_notifications()

    def send_notifications(self):
        """Send the notifications that are due, one frame per client connection."""
        now = time.monotonic()
        samples = collections.defaultdict(list)
        for handle, notification in self.notifications.items():
            if now < notification.start:
                continue
            if notification.mode == TRANS_SERVERCYCLE:
                if now < notification.next and not notification.pending:
                    continue
                notification.next = max(notification.next + notification.cycle, now)
            elif not notification.pending:
                continue
            notification.pending = False
            key = (notification.connection, notification.header)
            samples[key].append((handle, notification.variable.data))
        for (connection, header), items in samples.items():
            connection.send_notification(header, items)

    # ADS services

    def handle(self, connection, command, data, header):
        """Process a request, return (ADS result, response data)."""
        group = struct.unpack_from('<I', data)[0] if len(data) >= 4 else None
        self.requests[(command, group)] += 1
        if command == READ_DEVICE_INFO:
            return 0, struct.pack('<BBH16s', 3, 1, 4024, b'Plc30 App')
        if command == READ_STATE:
            return 0, struct.pack('<HH', 5 if self.running else 6, 0)  # RUN / STOP
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
            return self.add_notification(
                connection, group, offset, length, mode, cycle_time, header)
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
        if group == SYM_VERSION:
            return 0, bytes([self.symbol_version])[:length]
        variable = self.by_handle(group, offset)
        if variable is None:
            return (ERR_SYMBOLNOTFOUND if group == SYM_VALBYHND else ERR_INVALIDGRP), b''
        if length != len(variable.data):
            return ERR_INVALIDSIZE, b''
        return 0, variable.data

    def write_request(self, group, offset, value):
        if group == SYM_RELEASEHND:
            if len(value) != 4:
                return ERR_INVALIDSIZE
            handle, = struct.unpack('<I', value)
            with self.lock:
                found = self.handles.pop(handle, None)
            return 0 if found else ERR_SYMBOLNOTFOUND
        variable = self.by_handle(group, offset)
        if variable is None:
            return ERR_SYMBOLNOTFOUND if group == SYM_VALBYHND else ERR_INVALIDGRP
        if len(value) != len(variable.data):
            return ERR_INVALIDSIZE
        self.write(variable, value, client=True)
        return 0

    def read_write(self, group, offset, read_length, value):
        if group in (SUMUP_READ, SUMUP_WRITE, SUMUP_READWRITE):
            if not self.sum_commands:
                return ERR_INVALIDGRP, b''
            return self.sum_command(group, offset, read_length, value)
        if group in (SYM_HNDBYNAME, SYM_VALBYNAME, SYM_INFOBYNAMEEX):
            variable = self.by_name(value)
            if variable is None:
                return ERR_SYMBOLNOTFOUND, b''
            if group == SYM_VALBYNAME:
                return 0, variable.data[:read_length]
            if group == SYM_INFOBYNAMEEX:
                return self.symbol_info(variable, read_length)
            with self.lock:
                handle = self.next_handle
                self.next_handle += 1
                self.handles[handle] = variable
            return 0, struct.pack('<I', handle)
        return ERR_INVALIDGRP, b''

    def symbol_info(self, variable, read_length):
        """Return the AdsSymbolEntry of a variable."""
        name = variable.name.encode()
        type_name = variable.type_name.encode()
        strings = name + b'\0' + type_name + b'\0' + b'\0'
        entry = struct.pack(
            '<IIIIIIHHH', 30 + len(strings), 0x4040, 0, len(variable.data), 65, 8, len(name),
            len(type_name), 0) + strings
        if len(entry) > read_length:
            return ERR_INVALIDSIZE, b''
        return 0, entry

    def sum_command(self, group, count, read_length, data):
        """ADS sum command: `count` sub-commands in one request."""
        if count > 500:
            return ERR_INVALIDPARM, b''
        if group == SUMUP_READ:
            results, values = b'', b''
            for i in range(count):
                sub_group, sub_offset, length = struct.unpack_from('<III', data, 12 * i)
                error, value = self.read(sub_group, sub_offset, length)
                results += struct.pack('<I', error)
                values += value if error == 0 else bytes(length)
            return 0, results + values
        if group == SUMUP_WRITE:
            results, position = b'', 12 * count
            for i in range(count):
                sub_group, sub_offset, length = struct.unpack_from('<III', data, 12 * i)
                error = self.write_request(sub_group, sub_offset, data[position:position + length])
                position += length
                results += struct.pack('<I', error)
            return 0, results
        results, values, position = b'', b'', 16 * count
        for i in range(count):
            sub_group, sub_offset, sub_read, sub_write = struct.unpack_from('<IIII', data, 16 * i)
            sub_data = data[position:position + sub_write]
            position += sub_write
            error, value = self.read_write(sub_group, sub_offset, sub_read, sub_data)
            if error == 0 and len(value) > sub_read:
                error, value = ERR_INVALIDSIZE, b''
            if error != 0:
                value = b''
            results += struct.pack('<II', error, len(value))
            values += value
        return 0, results + values

    def add_notification(self, connection, group, offset, length, mode, cycle_time, header):
        variable = self.by_handle(group, offset)
        if variable is None:
            return ERR_SYMBOLNOTFOUND, b''
        if length != len(variable.data):
            return ERR_INVALIDSIZE, b''
        with self.lock:
            handle = self.next_notification
            self.next_notification += 1
            notification = Notification(connection, variable, header, mode, cycle_time)
            # like TwinCAT: the current value is sent after the response, in a next cycle
            notification.start = time.monotonic() + max(self.cycle, 0.002)
            self.notifications[handle] = notification
        return 0, struct.pack('<I', handle)


class AmsConnection(socketserver.BaseRequestHandler):
    """A client connection: AMS/TCP frames in, responses and notifications out."""

    def setup(self):
        self.send_lock = threading.Lock()
        self.closed = False
        self.requests = queue.Queue()
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
        processor = threading.Thread(target=self.process, daemon=True)
        processor.start()
        try:
            while True:
                _, length = struct.unpack('<HI', self.receive(6))
                frame = self.receive(length)
                self.requests.put((time.monotonic(), frame))
        except (ConnectionError, OSError):
            pass
        finally:
            self.closed = True
            self.requests.put(None)
            processor.join()
            with self.simulator.lock:
                self.simulator.connections.discard(self)
                for handle, notification in list(self.simulator.notifications.items()):
                    if notification.connection is self:
                        del self.simulator.notifications[handle]

    def process(self):
        """Process the requests in order, at the end of the PLC cycle they arrived in."""
        while True:
            item = self.requests.get()
            if item is None:
                return
            arrival, frame = item
            delay = self.simulator.next_cycle(arrival) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            (target_net_id, target_port, source_net_id, source_port, command, flags, _,
             _, invoke_id) = AMS_HEADER.unpack_from(frame)
            if flags & 0x0001:  # a response
                continue
            header = (source_net_id, source_port, target_net_id, target_port)
            with self.simulator.lock:
                running = self.simulator.running
            if target_port != PLC_PORT or not running:
                # answered by the AMS router: the target port does not exist
                self.send(AMS_HEADER.pack(
                    source_net_id, source_port, target_net_id, target_port, command, 0x0005, 0,
                    ERR_TARGETPORTNOTFOUND, invoke_id))
                continue
            with self.simulator.lock:
                error, data = self.simulator.handle(
                    self, command, frame[AMS_HEADER.size:], header)
            payload = struct.pack('<I', error) + (data if error == 0 else b'')
            if command in (READ, READ_WRITE) and error != 0:
                payload += struct.pack('<I', 0)
            self.send(AMS_HEADER.pack(
                source_net_id, source_port, target_net_id, target_port, command, 0x0005,
                len(payload), 0, invoke_id) + payload)

    def send_notification(self, header, samples):
        """Send samples [(handle, data)] in one device notification (one stamp)."""
        client_net_id, client_port, plc_net_id, plc_port = header
        stamp = struct.pack('<QI', filetime(), len(samples)) + b''.join(
            struct.pack('<II', handle, len(data)) + data for handle, data in samples)
        stamps = struct.pack('<I', 1) + stamp
        payload = struct.pack('<I', len(stamps)) + stamps
        self.send(AMS_HEADER.pack(
            client_net_id, client_port, plc_net_id, plc_port, NOTIFICATION, 0x0004,
            len(payload), 0, 0) + payload)


def control(simulator, line):
    """
    Execute a command of the standard input and return its answer.

    set NAME VALUE   changes a variable (VALUE is a Python literal)
    get NAME         value of a variable
    drop             closes the connections (like a reboot of the PLC)
    program          downloads a new program (the handles and notifications are lost)
    stop, start      stops / starts the PLC runtime
    sum on|off       enables / disables the sum commands
    requests         number of requests received for each (command, index group)
    writes           number of writes of variables by the clients
    """
    words = line.split(None, 2)
    if not words:
        return ''
    command = words[0]
    if command == 'set':
        simulator.set_value(words[1], ast.literal_eval(words[2]))
    elif command == 'get':
        return repr(simulator.get_value(words[1]))
    elif command == 'drop':
        simulator.drop_connections()
    elif command == 'program':
        simulator.change_program()
    elif command == 'stop':
        simulator.stop_runtime()
    elif command == 'start':
        simulator.start_runtime()
    elif command == 'sum':
        simulator.sum_commands = words[1] == 'on'
    elif command == 'requests':
        with simulator.lock:
            return json.dumps(
                {'%d,%s' % (cmd, '0x%X' % group if group is not None else '-'): count
                 for (cmd, group), count in simulator.requests.items()}, sort_keys=True)
    elif command == 'writes':
        return str(len(simulator.writes))
    else:
        return 'unknown command'
    return 'ok'


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    parser.add_argument('config', help='beckhoff_ros JSON configuration file')
    parser.add_argument('--host', help='address to listen on (default: from net_id)')
    parser.add_argument('--port', type=int, default=ADS_TCP_PORT, help='TCP port (default 48898)')
    parser.add_argument(
        '--cycle-ms', type=float, default=0.0,
        help='PLC cycle time: requests are processed at the end of the cycle (default: at once)')
    parser.add_argument(
        '--value', action='append', default=[], metavar='NAME=VALUE',
        help='initial value of a variable, as a Python literal')
    parser.add_argument(
        '--control', action='store_true', help='read commands from the standard input')
    args = parser.parse_args()
    with open(args.config) as config_file:
        plc_config = json.load(config_file)['PLC']
    values = {}
    for item in args.value:
        name, value = item.split('=', 1)
        values[name] = ast.literal_eval(value)
    simulator = AdsSimulator(
        plc_config, host=args.host, port=args.port, values=values, cycle_ms=args.cycle_ms).start()
    print('ADS simulator listening on %s:%d' % (simulator.host, simulator.port), flush=True)
    try:
        if args.control:
            for line in sys.stdin:
                if line.strip() == 'quit':
                    break
                try:
                    answer = control(simulator, line.strip())
                except Exception as error:  # noqa: B902 the error is the answer
                    answer = 'error: %s' % error
                print(answer, flush=True)
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    simulator.stop()


if __name__ == '__main__':
    main()
