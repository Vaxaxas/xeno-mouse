#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAKCU Hardware API Verification Example
Tests:
  1. MAKCU ASCII protocol commands (km.version, km.move, km.left, km.click, km.mo, km.echo)
  2. MAKCU Dot-prefix commands (.move, .left)
  3. MAKCU V2 binary protocol (0x50 frame)
  4. MAKCU 7-byte compact binary protocol (0xA5 frame with CRC8)
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bridge"))
from makcu import Makcu


def main():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    target_port = None
    for p in ports:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        if "CH340" in desc or "1A86:7523" in hwid:
            target_port = p.device
            break

    if not target_port:
        print("No CH340 COM port found. Please specify COM port.")
        return

    print(f"Connecting to MAKCU device on {target_port} @ 115200 baud...")
    dev = Makcu(target_port, 115200)

    print("\n--- 1. MAKCU ASCII API ---")
    print("Version query:", dev.version())
    print("Relative move (50, 50):", dev.move(50, 50))
    time.sleep(0.1)
    print("Dot-prefix move (-50, -50):", dev.send_cmd(".move(-50,-50)"))
    time.sleep(0.1)
    print("Left button getter:", dev.left())
    print("Left button click:", dev.click(1))
    print("Raw MO frame (buttons=0, dx=30, dy=30, wheel=0):", dev.mo(0, 30, 30, 0, 0, 0))

    print("\n--- 2. MAKCU V2 Binary API (0x50 frame) ---")
    ok_v2 = dev.send_v2_move(-30, -30)
    print("V2 Binary Move (-30, -30):", "OK" if ok_v2 else "Failed")

    print("\n--- 3. MAKCU 7-Byte Compact CRC8 Binary API (0xA5 frame) ---")
    ok_7b = dev.send_binary_7b(40, 20, 0)
    print("7-Byte CRC8 Move (40, 20):", "OK" if ok_7b else "Failed")
    time.sleep(0.05)
    dev.send_binary_7b(-40, -20, 0)

    print("\nAll MAKCU API tests completed successfully!")
    dev.close()


if __name__ == "__main__":
    main()
