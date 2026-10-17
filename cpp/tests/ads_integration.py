"""Exercise AdsLib over localhost using pyads' hardware-free ADS test server.

The upstream AdvancedHandler sends notifications only to in-process pyads
callbacks. This adapter emits the corresponding ADS packets to the C++ process
and accepts subscribe-by-handle, as real TwinCAT does. No PLC is contacted.
"""

import socket
import struct
import subprocess
import sys
import time

from pyads import constants
from pyads.testserver import AdsTestServer, AdvancedHandler, PLCVariable


class WireNotifications(AdvancedHandler):
    def __init__(self):
        super().__init__()
        self.server = None
        self.subscriptions = {}
        self.acquired = 0
        self.released = 0
        self.writes = 0

    def handle_request(self, request):
        header = request.ams_header
        command = struct.unpack("<H", header.command_id)[0]
        data = header.data
        changed_variable = None
        if command == constants.ADSCOMMAND_ADDDEVICENOTE:
            group, offset = struct.unpack_from("<II", data)
            if group == constants.ADSIGRP_SYM_VALBYHND:
                variable = self.get_variable_by_handle(offset)
                translated = struct.pack("<II", variable.index_group, variable.index_offset) + data[8:]
                request = request._replace(ams_header=header._replace(data=translated))
        elif command == constants.ADSCOMMAND_WRITE:
            group, offset = struct.unpack_from("<II", data)
            if group == constants.ADSIGRP_SYM_RELEASEHND:
                self.released += 1
            elif group == constants.ADSIGRP_SYM_VALBYHND:
                changed_variable = self.get_variable_by_handle(offset)
                self.writes += 1
        elif command == constants.ADSCOMMAND_READWRITE:
            group = struct.unpack_from("<I", data)[0]
            if group == constants.ADSIGRP_SYM_HNDBYNAME:
                self.acquired += 1

        response = super().handle_request(request)
        if command == constants.ADSCOMMAND_ADDDEVICENOTE:
            # AdvancedHandler prepends a four-byte ADS status to response.data.
            token = struct.unpack_from("<I", response.data, 4)[0]
            group, offset = struct.unpack_from("<II", request.ams_header.data)
            variable = self.get_variable_by_indices(group, offset)
            self.subscriptions[token] = (variable, header)
        elif command == constants.ADSCOMMAND_DELDEVICENOTE:
            token = struct.unpack_from("<I", data)[0]
            self.subscriptions.pop(token, None)
        elif changed_variable is not None:
            for token, (variable, subscription_header) in tuple(self.subscriptions.items()):
                if variable is changed_variable:
                    self.notify(token, variable.value, subscription_header)
        return response

    def notify(self, token, value, request_header):
        # One timestamp containing one sample. Stream length excludes itself.
        stamp = struct.pack("<QI", 0, 1) + struct.pack("<II", token, len(value)) + value
        payload = struct.pack("<II", 4 + len(stamp), 1) + stamp
        ams = (
            request_header.source_net_id + request_header.source_port
            + request_header.target_net_id + request_header.target_port
            + struct.pack("<HHIII", constants.ADSCOMMAND_DEVICENOTE, 4, len(payload), 0, 0)
            + payload
        )
        packet = struct.pack("<HI", 0, len(ams)) + ams
        for client in self.server.clients:
            if client.is_alive():
                client.client.sendall(packet)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: ads_integration.py /path/to/beckhoff_ads_probe")
    handler = WireNotifications()
    handler.add_variable(PLCVariable("GVL.counter", 42, constants.ADST_UINT32, "UDINT"))
    handler.add_variable(PLCVariable("GVL.status", b"\x07\x00\x01\x00", constants.ADST_BIGTYPE, "Status"))
    with AdsTestServer(handler=handler, ip_address="127.0.0.1", logging=False) as server:
        handler.server = server
        deadline = time.monotonic() + 2
        while not server.server.getsockopt(socket.SOL_SOCKET, socket.SO_ACCEPTCONN):
            if time.monotonic() >= deadline:
                raise TimeoutError("localhost ADS server did not start listening")
            time.sleep(0.005)
        subprocess.run([sys.argv[1]], check=True, timeout=15)
        if handler.acquired != 4 or handler.released != 4:
            raise AssertionError(f"ADS handle lifecycle mismatch: {handler.acquired} acquired, {handler.released} released")
        if handler.subscriptions:
            raise AssertionError("ADS notifications were not removed")
        if handler.writes != 2:
            raise AssertionError(f"expected two localhost symbol writes, got {handler.writes}")
    print("ADS server verified handle and subscription cleanup")


if __name__ == "__main__":
    main()
