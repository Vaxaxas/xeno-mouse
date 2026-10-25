#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build and Flash Script for Xeno Mouse (Arduino Leonardo + USB Host Shield)
Compiles with -DCDC_DISABLED and -mcall-prologues for minimal footprint and pure HID stealth.
Supports automatic remote bootloader trigger via km.boot() and manual physical RESET detection.
"""

import os
import subprocess
import sys
import time
import serial
import serial.tools.list_ports

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SKETCH_DIR = os.path.join(BASE_DIR, "firmware", "universal_passthrough")
BUILD_DIR = os.path.join(BASE_DIR, "build")


def find_arduino_tools():
    arduino_cli = r"C:\Users\fi\.arduino-tools\arduino-cli.exe"
    if not os.path.exists(arduino_cli):
        arduino_cli = "arduino-cli"

    avrdude = r"C:\Users\fi\AppData\Local\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe"
    conf = r"C:\Users\fi\AppData\Local\Arduino15\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf"
    return arduino_cli, avrdude, conf


def compile_firmware():
    arduino_cli, _, _ = find_arduino_tools()
    print("Compiling universal_passthrough.ino with -DCDC_DISABLED and -mcall-prologues...")
    os.makedirs(BUILD_DIR, exist_ok=True)
    cmd = [
        arduino_cli, "compile",
        "--fqbn", "arduino:avr:leonardo",
        "--build-property", "compiler.c.extra_flags=-DCDC_DISABLED -mcall-prologues",
        "--build-property", "compiler.cpp.extra_flags=-DCDC_DISABLED -mcall-prologues",
        "--output-dir", BUILD_DIR,
        SKETCH_DIR
    ]
    res = subprocess.run(cmd, text=True)
    if res.returncode != 0:
        print("Compilation failed!")
        sys.exit(res.returncode)
    print("Compilation succeeded!\n")


def flash_firmware():
    _, avrdude, conf = find_arduino_tools()
    hex_file = os.path.join(BUILD_DIR, "universal_passthrough.ino.hex")
    if not os.path.exists(hex_file):
        print(f"Hex file not found at {hex_file}")
        sys.exit(1)

    # 1. Try remote bootloader entry via CH340
    for p in serial.tools.list_ports.comports():
        if "1A86:7523" in (p.hwid or "").upper() or "CH340" in (p.description or "").upper():
            print(f"Attempting remote km.boot() on {p.device}...")
            try:
                s = serial.Serial(p.device, 115200, timeout=0.1)
                s.write(b"\x7c\r\nkm.boot()\r\n")
                s.flush()
                s.close()
                time.sleep(0.3)
            except Exception:
                pass

    # 2. Wait for Caterina bootloader (2341:0036)
    print("Waiting for Leonardo Caterina bootloader (2341:0036)...")
    boot_port = None
    for _ in range(100):
        for p in serial.tools.list_ports.comports():
            if "2341:0036" in (p.hwid or "").upper():
                boot_port = p.device
                break
        if boot_port:
            break
        time.sleep(0.1)

    if not boot_port:
        print("Bootloader port not detected. Please press the physical RESET button on Leonardo.")
        sys.exit(1)

    print(f"Flashing on {boot_port}...")
    time.sleep(0.3)
    cmd = [
        avrdude, "-C", conf, "-v", "-patmega32u4", "-cavr109",
        f"-P{boot_port}", "-b57600", "-D", f"-Uflash:w:{hex_file}:i"
    ]
    res = subprocess.run(cmd)
    if res.returncode == 0:
        print("\nFlashing completed successfully!")
    else:
        print("\nFlashing failed.")
        sys.exit(res.returncode)


if __name__ == "__main__":
    compile_firmware()
    if "--flash" in sys.argv:
        flash_firmware()
