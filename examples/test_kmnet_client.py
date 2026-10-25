#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standard kmbox Net UDP Client & Verification Test Suite
Compatible with 127.0.0.1:50050 (UUID: 12345678)
"""

import socket
import struct
import time

CMD_CONNECT        = 0xAF3C2828
CMD_MOUSE_MOVE     = 0xAEDE7345
CMD_MOUSE_LEFT     = 0x9823AE8D
CMD_MOUSE_MIDDLE   = 0x97A3AE8D
CMD_MOUSE_RIGHT    = 0x238D8212
CMD_MOUSE_WHEEL    = 0xFFEEAD38
CMD_MOUSE_AUTOMOVE = 0xAEDE7346
CMD_BAZER_MOVE     = 0xA238455A
CMD_MOUSE_ALL      = 0xAEDE7347
CMD_MONITOR        = 0x27388020
CMD_MASK_MOUSE     = 0x23234343
CMD_UNMASK_ALL     = 0x23344343


class KmNetClient:
    def __init__(self, ip="127.0.0.1", port=50050, uuid="12345678"):
        self.addr = (ip, int(port))
        self.mac = int(str(uuid).strip(), 16) & 0xFFFFFFFF
        self.indexpts = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def _send(self, cmd, rand_val=0, payload=b""):
        self.indexpts = (self.indexpts + 1) & 0xFFFFFFFF
        hdr = struct.pack("<IIII", self.mac, rand_val & 0xFFFFFFFF, self.indexpts, cmd & 0xFFFFFFFF)
        self.sock.sendto(hdr + payload, self.addr)
        resp, _ = self.sock.recvfrom(64)
        return len(resp) >= 16

    def init(self):
        return self._send(CMD_CONNECT, 0)

    def move(self, x: int, y: int):
        payload = struct.pack("<iiii", 0, int(x), int(y), 0)
        return self._send(CMD_MOUSE_MOVE, 0, payload)

    def move_auto(self, x: int, y: int, ms: int):
        payload = struct.pack("<iiii", 0, int(x), int(y), 0)
        return self._send(CMD_MOUSE_AUTOMOVE, int(ms), payload)

    def left(self, is_down: int):
        payload = struct.pack("<iiii", 1 if is_down else 0, 0, 0, 0)
        return self._send(CMD_MOUSE_LEFT, 0, payload)

    def right(self, is_down: int):
        payload = struct.pack("<iiii", 2 if is_down else 0, 0, 0, 0)
        return self._send(CMD_MOUSE_RIGHT, 0, payload)

    def middle(self, is_down: int):
        payload = struct.pack("<iiii", 4 if is_down else 0, 0, 0, 0)
        return self._send(CMD_MOUSE_MIDDLE, 0, payload)

    def wheel(self, w: int):
        payload = struct.pack("<iiii", 0, 0, 0, int(w))
        return self._send(CMD_MOUSE_WHEEL, 0, payload)

    def mask(self, bitmask: int):
        return self._send(CMD_MASK_MOUSE, bitmask & 0xFF)

    def unmask_all(self):
        return self._send(CMD_UNMASK_ALL, 0)


if __name__ == "__main__":
    cli = KmNetClient("127.0.0.1", 50050, "12345678")
    print("kmNet_init:", cli.init())
    print("kmNet_mouse_move(40, 20):", cli.move(40, 20))
    time.sleep(0.05)
    print("kmNet_mouse_move(-40, -20):", cli.move(-40, -20))
