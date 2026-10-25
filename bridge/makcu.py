#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAKCU Hardware API Client (Python)
Compatible with https://makcu.com/en/api/ specifications.
Supports:
  1. MAKCU ASCII protocol (km.cmd(...) / .cmd(...))
  2. MAKCU V2 binary protocol (0x50 frame)
  3. MAKCU 7-byte compact binary protocol (0xA5 frame with CRC8)
  4. MAKCU V4 / DE AD baud rate switching
"""

import struct
import time
import serial


def compute_crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


class Makcu:
    def __init__(self, port=None, baud=115200, timeout=0.1):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        if port:
            self.connect(port, baud)

    def connect(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baudrate=baud, timeout=self.timeout)
        time.sleep(0.05)
        return self.ser.is_open

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_cmd(self, cmd_str: str) -> str:
        """Send an ASCII command and read response."""
        if not self.ser or not self.ser.is_open:
            return ""
        if not cmd_str.endswith("\r\n"):
            cmd_str += "\r\n"
        self.ser.reset_input_buffer()
        self.ser.write(cmd_str.encode("ascii"))
        resp = self.ser.read_until(b">>> ")
        if not resp:
            resp = self.ser.read(64)
        return resp.decode("ascii", errors="ignore").strip()

    # -------------------------------------------------------------------------
    # ASCII API Methods (https://makcu.com/en/api/)
    # -------------------------------------------------------------------------
    def move(self, dx: int, dy: int) -> str:
        """Relative mouse move: km.move(dx, dy)"""
        return self.send_cmd(f"km.move({int(dx)},{int(dy)})")

    def move_now(self, dx: int, dy: int) -> str:
        """Immediate relative mouse move: km.move_now(dx, dy)"""
        return self.send_cmd(f"km.move_now({int(dx)},{int(dy)})")

    def mo(self, buttons: int, x: int, y: int, wheel: int = 0, pan: int = 0, tilt: int = 0) -> str:
        """Raw low-level mouse frame: km.mo(buttons, x, y, wheel, pan, tilt)"""
        return self.send_cmd(f"km.mo({int(buttons)},{int(x)},{int(y)},{int(wheel)},{int(pan)},{int(tilt)})")

    def left(self, state: int = None) -> str:
        """Left button SET (0=release, 1=press) or GET (state=None)"""
        if state is None:
            return self.send_cmd("km.left()")
        return self.send_cmd(f"km.left({int(state)})")

    def right(self, state: int = None) -> str:
        """Right button SET (0=release, 1=press) or GET (state=None)"""
        if state is None:
            return self.send_cmd("km.right()")
        return self.send_cmd(f"km.right({int(state)})")

    def middle(self, state: int = None) -> str:
        """Middle button SET (0=release, 1=press) or GET (state=None)"""
        if state is None:
            return self.send_cmd("km.middle()")
        return self.send_cmd(f"km.middle({int(state)})")

    def side1(self, state: int = None) -> str:
        """Side 1 button SET or GET"""
        if state is None:
            return self.send_cmd("km.side1()")
        return self.send_cmd(f"km.side1({int(state)})")

    def side2(self, state: int = None) -> str:
        """Side 2 button SET or GET"""
        if state is None:
            return self.send_cmd("km.side2()")
        return self.send_cmd(f"km.side2({int(state)})")

    def wheel(self, delta: int) -> str:
        """Scroll wheel: km.wheel(delta)"""
        return self.send_cmd(f"km.wheel({int(delta)})")

    def click(self, button: int = 1) -> str:
        """Click button (1=left, 2=right, 3=middle, 4=side1, 5=side2)"""
        return self.send_cmd(f"km.click({int(button)})")

    def version(self) -> str:
        """Query firmware version: km.version()"""
        return self.send_cmd("km.version()")

    def echo(self, enable: int) -> str:
        """Command echo enable/disable (1=enable, 0=disable)"""
        return self.send_cmd(f"km.echo({int(enable)})")

    def init(self) -> str:
        """Reset injected state: km.init()"""
        return self.send_cmd("km.init()")

    def unmask_all(self) -> str:
        """Clear all button masks: km.unmask_all()"""
        return self.send_cmd("km.unmask_all()")

    def baud(self, rate: int) -> str:
        """Switch baud rate: km.baud(rate)"""
        resp = self.send_cmd(f"km.baud({int(rate)})")
        if rate > 0 and self.ser and self.ser.is_open:
            self.ser.flush()
            self.ser.baudrate = rate
            self.baud = rate
        return resp

    def reboot(self) -> str:
        """Reboot device / enter bootloader: km.reboot()"""
        return self.send_cmd("km.reboot()")

    # -------------------------------------------------------------------------
    # Binary API Methods
    # -------------------------------------------------------------------------
    def send_binary_7b(self, dx: int, dy: int, buttons: int = 0) -> bool:
        """
        Fast 7-byte CRC8 binary packet (0xA5):
        [Header: 0xA5, dx: i16, dy: i16, buttons: u8, crc8: u8]
        Matches MakcuBinaryBackend in ai-colorbot.
        """
        if not self.ser or not self.ser.is_open:
            return False
        dx = int(max(-32767, min(32767, dx)))
        dy = int(max(-32767, min(32767, dy)))
        raw = struct.pack("<BhhB", 0xA5, dx, dy, buttons & 0x1F)
        crc = compute_crc8(raw)
        pkt = raw + bytes([crc])
        return self.ser.write(pkt) == 7

    def send_v2_move(self, dx: int, dy: int) -> bool:
        """
        MAKCU V2 binary move packet (0x50 frame, cmd 0x0D):
        [0x50, 0x0D, 0x04, 0x00, dx: i16, dy: i16]
        """
        if not self.ser or not self.ser.is_open:
            return False
        dx = int(max(-32767, min(32767, dx)))
        dy = int(max(-32767, min(32767, dy)))
        pkt = struct.pack("<BBHHhh", 0x50, 0x0D, 4, 0, dx, dy)
        self.ser.write(pkt[:4] + struct.pack("<hh", dx, dy))
        ack = self.ser.read(5)
        return len(ack) == 5 and ack[0] == 0x50 and ack[1] == 0x0D

    def send_v2_button(self, button_cmd: int, state: int) -> bool:
        """MAKCU V2 binary button packet (cmd: 0x08=left, 0x11=right, 0x0A=middle, 0x12=side1, 0x13=side2)"""
        if not self.ser or not self.ser.is_open:
            return False
        pkt = bytes([0x50, button_cmd & 0xFF, 1, 0, 1 if state else 0])
        self.ser.write(pkt)
        ack = self.ser.read(5)
        return len(ack) == 5 and ack[0] == 0x50 and ack[1] == button_cmd
