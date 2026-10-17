"""Exercise native AdsLib against a minimal, independent localhost ADS peer.

Only Python's standard library is needed. This is deliberately a narrow wire
fixture, not an ADS implementation: unexpected requests fail the test instead
of receiving a permissive success response. No PLC or remote host is contacted.
"""

from pathlib import Path
import socketserver
import struct
import subprocess
import sys
import threading


READ, WRITE, ADD_NOTIFICATION, DELETE_NOTIFICATION, NOTIFICATION, READ_WRITE = 2, 3, 6, 7, 8, 9
HANDLE_BY_NAME, VALUE_BY_HANDLE, RELEASE_HANDLE = 0xF003, 0xF005, 0xF006
AMS_HEADER = struct.Struct("<6sH6sHHHIII")
PLC_ADDRESS = (bytes((127, 0, 0, 1, 1, 2)), 851)
CLIENT_NET_ID = bytes((127, 0, 0, 1, 1, 1))
COMPLEX_NAME = "GVL.complex"


def golden_test_struct3():
    """Build the PLC bytes independently, without using the C++ codec or JSON.

    Default two-byte alignment: root fields at 0, 2, 4, 436; the six
    TestStruct2 elements have 72-byte strides and ten 7-byte leaf structures.
    Each leaf encodes a distinct seven-bit identifier in its bool fields.
    """
    result = bytearray(444)
    result[0] = 1
    struct.pack_into("<H", result, 2, 0xA35C)

    def leaf(offset, group, element):
        identifier = (group * 10 + element) ^ 0x55
        for bit in range(7):
            result[offset + bit] = (identifier >> bit) & 1

    for group in range(6):
        base = 4 + group * 72
        for element in range(10):
            leaf(base + element * 7, group, element)
        struct.pack_into("<H", result, base + 70, 0x2100 + group * 0x0137)
    leaf(436, 6, 0)
    return bytes(result)


class ProtocolError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise ProtocolError(message)


def packet(header, command, data, response):
    target_net, target_port, source_net, source_port, _, _, _, _, invoke = header
    ams = AMS_HEADER.pack(
        source_net, source_port, target_net, target_port, command,
        5 if response else 4, len(data), 0, invoke if response else 0,
    ) + data
    return struct.pack("<HI", 0, len(ams)) + ams


class AdsPeer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self):
        self.lock = threading.RLock()
        self.values = {"GVL.counter": struct.pack("<I", 42), COMPLEX_NAME: bytes(444)}
        self.handles = {}  # ((source Net ID, local ADS port), handle) -> symbol
        self.subscriptions = {}  # ((source Net ID, local ADS port), token) -> subscription
        self.next_handle = 1
        self.next_notification = {}
        self.acquired = self.released = self.writes = self.complex_writes = 0
        self.errors = []
        super().__init__(("127.0.0.1", 48898), AdsConnection)

    def variable(self, owner, handle):
        require((owner, handle) in self.handles, "handle does not belong to the requesting ADS port")
        return self.handles[owner, handle]

    def handle(self, connection, header, data):
        target_net, target_port, source_net, source_port, command, flags, length, error, invoke = header
        require((target_net, target_port) == PLC_ADDRESS, "unexpected destination AMS address")
        require(source_net == CLIENT_NET_ID and source_port != 0, "unexpected client AMS address")
        require(flags == 4 and error == 0 and invoke != 0 and length == len(data), "invalid AMS request header")
        owner = source_net, source_port
        notifications = []

        if command == READ_WRITE:
            require(len(data) >= 16, "truncated read/write request")
            group, offset, read_size, write_size = struct.unpack_from("<IIII", data)
            require(group == HANDLE_BY_NAME and offset == 0 and read_size == 4,
                    "only four-byte handle acquisition by name is supported")
            name = data[16:]
            require(write_size == len(name) and name.endswith(b"\0") and b"\0" not in name[:-1],
                    "invalid symbol name buffer")
            name = name[:-1].decode("ascii")
            require(name in self.values, "unknown ADS symbol")
            handle = self.next_handle
            self.next_handle += 1
            self.handles[owner, handle] = name
            self.acquired += 1
            response = struct.pack("<III", 0, 4, handle)
        elif command == READ:
            require(len(data) == 12, "invalid ADS read request size")
            group, offset, read_size = struct.unpack("<III", data)
            require(group == VALUE_BY_HANDLE, "unsupported read index group")
            value = self.values[self.variable(owner, offset)]
            require(read_size == len(value), "read size does not match PLC type")
            response = struct.pack("<II", 0, len(value)) + value
        elif command == WRITE:
            require(len(data) >= 12, "truncated ADS write request")
            group, offset, write_size = struct.unpack_from("<III", data)
            value = data[12:]
            require(write_size == len(value), "ADS write payload length mismatch")
            if group == RELEASE_HANDLE:
                require(offset == 0 and write_size == 4, "invalid symbol handle release")
                handle = struct.unpack("<I", value)[0]
                self.variable(owner, handle)
                require(not any(key[0] == owner and subscription[0] == handle
                                for key, subscription in self.subscriptions.items()),
                        "symbol handle released before its notification")
                del self.handles[owner, handle]
                self.released += 1
            else:
                require(group == VALUE_BY_HANDLE, "unsupported write index group")
                name = self.variable(owner, offset)
                require(write_size == len(self.values[name]), "write size does not match PLC type")
                if name == COMPLEX_NAME:
                    require(self.complex_writes < 2, "unexpected additional structure write")
                    expected = bytearray(golden_test_struct3())
                    if self.complex_writes == 1:
                        expected[0] = 0
                    require(value == expected, "TestStruct3 wire bytes differ from independent PLC layout")
                    self.complex_writes += 1
                else:
                    require(value == struct.pack("<I", 123456), "unexpected counter wire value")
                self.values[name] = value
                self.writes += 1
                for (subscriber, token), subscription in self.subscriptions.items():
                    _, subscribed_name, request_header, subscriber_connection = subscription
                    if subscribed_name == name:
                        # Route each packet to its subscriber's ADS port, even
                        # when clients share a TCP route and notification token.
                        stamp = struct.pack("<QI", 0, 1) + struct.pack("<II", token, len(value)) + value
                        payload = struct.pack("<II", 4 + len(stamp), 1) + stamp
                        notifications.append((subscriber_connection, packet(request_header, NOTIFICATION, payload, False)))
            response = struct.pack("<I", 0)
        elif command == ADD_NOTIFICATION:
            require(len(data) == 40, "invalid add-notification request size")
            group, handle, size, mode, _, _ = struct.unpack_from("<IIIIII", data)
            require(group == VALUE_BY_HANDLE and mode == 4 and data[24:] == bytes(16),
                    "only server-on-change notifications by handle are supported")
            name = self.variable(owner, handle)
            require(size == len(self.values[name]), "notification size does not match PLC type")
            token = self.next_notification.get(owner, 1)
            self.next_notification[owner] = token + 1
            self.subscriptions[owner, token] = handle, name, header, connection
            response = struct.pack("<II", 0, token)
        elif command == DELETE_NOTIFICATION:
            require(len(data) == 4, "invalid delete-notification request size")
            token = struct.unpack("<I", data)[0]
            require((owner, token) in self.subscriptions, "notification does not belong to requesting ADS port")
            del self.subscriptions[owner, token]
            response = struct.pack("<I", 0)
        else:
            raise ProtocolError(f"unsupported ADS command {command}")

        connection.send(packet(header, command, response, True))
        for subscriber_connection, notification in notifications:
            subscriber_connection.send(notification)

    def verify(self):
        require(not self.errors, f"localhost peer errors: {self.errors}")
        require(self.acquired == self.released == 4,
                f"ADS handle lifecycle mismatch: {self.acquired} acquired, {self.released} released")
        require(not self.handles and not self.subscriptions, "ADS handles or subscriptions leaked")
        require(len(self.next_notification) == 2, "expected two independent local ADS ports")
        require(self.writes == 3 and self.complex_writes == 2, "expected two complex writes and one scalar write")


class AdsConnection(socketserver.BaseRequestHandler):
    def setup(self):
        self.request.settimeout(5)
        self.send_lock = threading.Lock()

    def receive(self, size, allow_eof=False):
        result = bytearray()
        while len(result) < size:
            chunk = self.request.recv(size - len(result))
            if not chunk and allow_eof and not result:
                return None
            require(bool(chunk), "connection closed within an ADS packet")
            result.extend(chunk)
        return bytes(result)

    def send(self, data):
        with self.send_lock:
            self.request.sendall(data)

    def handle(self):
        try:
            while True:
                prefix = self.receive(6, allow_eof=True)
                if prefix is None:
                    return
                reserved, size = struct.unpack("<HI", prefix)
                require(reserved == 0 and AMS_HEADER.size <= size <= 4096, "invalid AMS/TCP frame length")
                frame = self.receive(size)
                header = AMS_HEADER.unpack_from(frame)
                with self.server.lock:
                    self.server.handle(self, header, frame[AMS_HEADER.size:])
        except (ProtocolError, OSError, UnicodeError, struct.error) as error:
            with self.server.lock:
                self.server.errors.append(str(error))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: ads_integration.py /path/to/beckhoff_ads_probe")
    parameters = Path(__file__).resolve().parents[1] / "examples" / "test_struct3.json"
    # TCPServer binds and listens synchronously; no readiness connection can be
    # mistaken for an ADS client and no startup delay is needed.
    with AdsPeer() as server:
        serving = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        serving.start()
        try:
            subprocess.run([sys.argv[1], str(parameters)], check=True, timeout=15)
            with server.lock:
                server.verify()
        finally:
            server.shutdown()
            serving.join(timeout=2)
            require(not serving.is_alive(), "localhost peer did not stop")
    print("Independent ADS peer verified TestStruct3 bytes, two-client notifications, and resource cleanup")


if __name__ == "__main__":
    main()
