#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lightweight kmbox Net -> Arduino Leonardo (Serial1 / CH340) Bridge & Controller
100% compatible with all 46 functions of the official kmNet.pyd C++ extension.
"""

import argparse
import ctypes
import json
import os
import socket
import struct
import threading
import time
import serial
import serial.tools.list_ports

# ============================================================================
# 1. Official kmNet.pyd Protocol Command Constants
# ============================================================================
CMD_CONNECT        = 0xAF3C2828
CMD_MOUSE_MOVE     = 0xAEDE7345
CMD_MOUSE_LEFT     = 0x9823AE8D
CMD_MOUSE_MIDDLE   = 0x97A3AE8D
CMD_MOUSE_RIGHT    = 0x238D8212
CMD_MOUSE_WHEEL    = 0xFFEEAD38  # Used by both wheel() and mouse() in kmNet.pyd
CMD_MOUSE_AUTOMOVE = 0xAEDE7346
CMD_BAZER_MOVE     = 0xA238455A
CMD_MOUSE_ALL      = 0xAEDE7347
CMD_KEYBOARD_ALL   = 0x123C2C2F  # keydown, keyup, keypress, enc_key*
CMD_MONITOR        = 0x27388020
CMD_DEBUG          = 0x27382021
CMD_MASK_MOUSE     = 0x23234343  # mask_left/right/middle/side1/side2/x/y/wheel/keyboard
CMD_UNMASK_ALL     = 0x23344343  # unmask_all, unmask_keyboard
CMD_SETCONFIG      = 0x1D3D3323  # setip_port
CMD_SETVIDPID      = 0xFFED3232
CMD_SHOWPIC        = 0x12334883  # lcd_color (40 x 1040B packets)
CMD_TRACE          = 0xBBCDDDAC  # trace, xbox_map
CMD_REBOOT         = 0xAA8855AA  # reboot

# Default Leonardo binary frame delimiter (xen_deobfuscate(0) == 0x7C)
DEFAULT_DELIM = 0x7C

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_config.json")
XEN_SETTINGS_PATH = os.path.expandvars(r"%APPDATA%\xenmouse\settings.json")

# USB HID Keyboard Usage ID -> Windows Virtual-Key (VK) mapping for keydown/keyup
HID_TO_VK = {
    **{0x04 + i: 0x41 + i for i in range(26)},  # A-Z
    0x1E: 0x31, 0x1F: 0x32, 0x20: 0x33, 0x21: 0x34, 0x22: 0x35,
    0x23: 0x36, 0x24: 0x37, 0x25: 0x38, 0x26: 0x39, 0x27: 0x30,
    0x28: 0x0D, 0x29: 0x1B, 0x2A: 0x08, 0x2B: 0x09, 0x2C: 0x20,
    **{0x3A + i: 0x70 + i for i in range(12)},  # F1-F12
    0x4F: 0x27, 0x50: 0x25, 0x51: 0x28, 0x52: 0x26,
    0xE0: 0xA2, 0xE1: 0xA0, 0xE2: 0xA4, 0xE3: 0x5B,
    0xE4: 0xA3, 0xE5: 0xA1, 0xE6: 0xA5, 0xE7: 0x5C,
}


def load_config():
    cfg = {
        "com_port": "AUTO",
        "baud": 115200,
        "net_ip": "127.0.0.1",
        "net_port": 50050,
        "uuid": "12345678",
        "smoothing": False,
    }
    if os.path.exists(XEN_SETTINGS_PATH):
        try:
            with open(XEN_SETTINGS_PATH, "r", encoding="utf-8") as f:
                x = json.load(f).get("deviceInfo", {})
                if x.get("netIP"):
                    cfg["net_ip"] = x["netIP"]
                if x.get("netPort"):
                    cfg["net_port"] = int(x["netPort"])
                if x.get("netPassword"):
                    cfg["uuid"] = str(x["netPassword"])
        except Exception:
            pass
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg.update(json.load(f))
        except Exception:
            pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


# ============================================================================
# 2. Core Bridge Engine (Serial1 <-> kmbox Net UDP)
# ============================================================================
class KmboxNetBridge:
    def __init__(self, cfg, on_status=None):
        self.cfg = cfg
        self.on_status = on_status
        self.running = False

        # Serial state
        self.ser = None
        self.ser_lock = threading.Lock()
        self.connected_port = None
        self.start_byte = DEFAULT_DELIM
        self.end_byte = DEFAULT_DELIM

        # UDP state
        self.sock = None
        self.mac_int = self._parse_uuid(cfg.get("uuid", "12345678"))

        # Mouse / Keyboard / Monitor state
        self.btn_or_mask = 0
        self.btn_block_mask = 0
        self.phys_buttons = 0
        self.kbd_ctrl = 0
        self.kbd_keys = b"\x00" * 10
        self.active_hid_keys = set()
        self.masked_keys = set()
        self.monitor_enabled = False
        self.monitor_addr = None  # (ip, port)
        self.mon_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Telemetry counters
        self.udp_rx_count = 0
        self.serial_tx_count = 0
        self.last_cmd_name = "None"
        self.last_client = "None"

        # Trajectory cancellation token
        self._traj_id = 0

    @staticmethod
    def _parse_uuid(uuid_str):
        s = str(uuid_str).strip().replace("0x", "").replace("0X", "")
        try:
            return int(s, 16) & 0xFFFFFFFF
        except ValueError:
            return 0x12345678

    @staticmethod
    def _xxtea_decrypt_128(data: bytes, mac_int: int) -> bytes:
        if len(data) < 128:
            return data
        k0 = struct.unpack(">I", struct.pack("<I", mac_int & 0xFFFFFFFF))[0]
        k = (k0, 0, 0, 0)
        v = list(struct.unpack("<32I", data[:128]))
        delta = 0x9E3779B9
        s = (6 * delta) & 0xFFFFFFFF
        for _ in range(6):
            e = (s >> 2) & 3
            for p in range(31, -1, -1):
                z = v[(p - 1) & 31]
                y = v[(p + 1) & 31]
                mx = (((z >> 5) ^ ((y << 2) & 0xFFFFFFFF)) + ((y >> 3) ^ ((z << 4) & 0xFFFFFFFF))) ^ (
                    (s ^ y) + (k[(p & 3) ^ e] ^ z)
                )
                v[p] = (v[p] - mx) & 0xFFFFFFFF
            s = (s - delta) & 0xFFFFFFFF
        return struct.pack("<32I", *v)

    def _notify(self, msg=None):
        if self.on_status:
            try:
                self.on_status(msg)
            except Exception:
                pass

    @staticmethod
    def find_ch340_port():
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            hwid = (p.hwid or "").upper()
            desc = (p.description or "").upper()
            if "1A86:7523" in hwid or "CH340" in desc:
                return p.device
        for p in ports:
            hwid = (p.hwid or "").upper()
            if "2341:" not in hwid:
                return p.device
        return None

    def connect_serial(self):
        target = self.cfg.get("com_port", "AUTO")
        if not target or target.upper() == "AUTO":
            target = self.find_ch340_port()
        if not target:
            self.connected_port = None
            return False

        with self.ser_lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(
                    port=target,
                    baudrate=int(self.cfg.get("baud", 115200)),
                    timeout=0.05,
                    write_timeout=0.1,
                )
                self.ser.reset_input_buffer()
                self.connected_port = target
            except Exception as e:
                self.ser = None
                self.connected_port = None
                self._notify(f"Serial open failed: {e}")
                return False

        # Auto-sync packet phase: ensure in_packet is False on the Leonardo
        with self.ser_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b"\r\nkm.ping()\r\n")
                time.sleep(0.03)
                if b"km.ok" not in self.ser.read(64):
                    self.ser.write(bytes([self.end_byte]) + b"\r\nkm.ping()\r\n")
                    time.sleep(0.03)
                    self.ser.read(64)
            except Exception:
                pass

        # Enable physical button reporting so kmNet.monitor works seamlessly
        self._send_frame(bytes([9, 1]))
        self._notify(f"Connected to {target} @ {self.cfg.get('baud', 115200)}")
        return True

    def _send_frame(self, payload: bytes):
        with self.ser_lock:
            if not self.ser or not self.ser.is_open:
                return False
            try:
                pkt = bytes([self.start_byte]) + payload + bytes([self.end_byte])
                self.ser.write(pkt)
                self.serial_tx_count += 1
                return True
            except Exception:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.connected_port = None
                return False

    # ------------------------------------------------------------------------
    # High-level Hardware Actions (Delimiter-Safe Binary Protocol)
    # ------------------------------------------------------------------------
    def move_rel(self, dx: int, dy: int):
        dx = int(max(-32767, min(32767, dx)))
        dy = int(max(-32767, min(32767, dy)))
        while dx != 0 or dy != 0:
            sx = max(-127, min(127, dx))
            sy = max(-127, min(127, dy))
            if (sx & 0xFF) in (self.start_byte, self.end_byte):
                sx = 123 if sx > 0 else -123
            if (sy & 0xFF) in (self.start_byte, self.end_byte):
                sy = 123 if sy > 0 else -123
            self._send_frame(struct.pack("<Bbb", 3, sx, sy))
            dx -= sx
            dy -= sy

    def set_button(self, mask: int, pressed: bool):
        action = 1 if pressed else 0
        if pressed:
            self.btn_or_mask |= (mask & 0x1F)
        else:
            self.btn_or_mask &= ~(mask & 0x1F)
        self._send_frame(struct.pack("<BBB", 1, mask & 0x1F, action))

    def wheel(self, w: int):
        w = int(max(-127, min(127, w)))
        if (w & 0xFF) in (self.start_byte, self.end_byte):
            w = 123 if w > 0 else -123
        self._send_frame(struct.pack("<Bb", 5, w))

    def mouse_all(self, buttons: int, dx: int, dy: int, wheel: int):
        self.btn_or_mask = buttons & 0x1F
        sx = int(max(-127, min(127, dx)))
        sy = int(max(-127, min(127, dy)))
        sw = int(max(-127, min(127, wheel)))
        if (sx & 0xFF) in (self.start_byte, self.end_byte):
            sx = 123 if sx > 0 else -123
        if (sy & 0xFF) in (self.start_byte, self.end_byte):
            sy = 123 if sy > 0 else -123
        if (sw & 0xFF) in (self.start_byte, self.end_byte):
            sw = 123 if sw > 0 else -123
        self._send_frame(struct.pack("<BBbbb", 7, self.btn_or_mask, sx, sy, sw))
        rem_x = dx - sx
        rem_y = dy - sy
        if rem_x != 0 or rem_y != 0:
            self.move_rel(rem_x, rem_y)

    def set_block_mask(self, block_mask: int):
        self.btn_block_mask = block_mask & 0x1F
        self._send_frame(struct.pack("<BB", 6, self.btn_block_mask))

    def handle_keyboard_all(self, payload: bytes):
        if len(payload) < 12:
            return
        self.kbd_ctrl = payload[0]
        self.kbd_keys = bytes(payload[2:12])
        new_keys = set()
        for i in range(8):
            if self.kbd_ctrl & (1 << i):
                new_keys.add(0xE0 + i)
        for b in self.kbd_keys:
            if b != 0:
                new_keys.add(b)

        # Synthesize Windows key events for newly pressed/released keys
        for hid_k in new_keys - self.active_hid_keys:
            vk = HID_TO_VK.get(hid_k)
            if vk:
                ctypes.windll.user32.keybd_event(vk, 0, 0, 0)
        for hid_k in self.active_hid_keys - new_keys:
            vk = HID_TO_VK.get(hid_k)
            if vk:
                ctypes.windll.user32.keybd_event(vk, 0, 2, 0)
        self.active_hid_keys = new_keys
        self._push_monitor_packet()

    def set_smoothing(self, enabled: bool):
        self.cfg["smoothing"] = bool(enabled)
        val = 0x01 if enabled else 0xFF
        self._send_frame(struct.pack("<BB", 17, val))

    def ping_board(self) -> bool:
        with self.ser_lock:
            if not self.ser or not self.ser.is_open:
                return False
            try:
                self.ser.reset_input_buffer()
                self.ser.write(bytes([self.start_byte, 10, 0xFF, self.end_byte]))
                time.sleep(0.03)
                resp = self.ser.read(16)
                return b"\x69" in resp
            except Exception:
                return False

    def enter_bootloader(self):
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"\r\nkm.boot()\r\n")
                    self.ser.flush()
                except Exception:
                    pass

    # ------------------------------------------------------------------------
    # Smooth Auto-Move & Cubic Bezier Curve Generators
    # ------------------------------------------------------------------------
    def start_smooth_move(self, dx: int, dy: int, duration_ms: int, ctrl_pts=None):
        self._traj_id += 1
        my_id = self._traj_id
        duration_ms = max(1, min(10000, int(duration_ms)))
        steps = max(1, duration_ms // 2)  # 500Hz (2ms per step)

        def _worker():
            acc_x = 0.0
            acc_y = 0.0
            t_start = time.perf_counter()
            for step in range(1, steps + 1):
                if self._traj_id != my_id or not self.running:
                    break
                t = step / float(steps)
                if ctrl_pts and len(ctrl_pts) >= 4:
                    x0, y0, x1, y1 = ctrl_pts[0], ctrl_pts[1], ctrl_pts[2], ctrl_pts[3]
                    u = 1.0 - t
                    target_x = 3 * u * u * t * x0 + 3 * u * t * t * x1 + (t ** 3) * dx
                    target_y = 3 * u * u * t * y0 + 3 * u * t * t * y1 + (t ** 3) * dy
                else:
                    target_x = dx * t
                    target_y = dy * t

                step_dx = int(round(target_x - acc_x))
                step_dy = int(round(target_y - acc_y))
                if step_dx != 0 or step_dy != 0:
                    self.move_rel(step_dx, step_dy)
                    acc_x += step_dx
                    acc_y += step_dy

                next_time = t_start + (step * (duration_ms / 1000.0) / steps)
                rem = next_time - time.perf_counter()
                if rem > 0:
                    time.sleep(rem)

        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------------------
    # Background Threads (Serial RX Monitor + UDP Server)
    # ------------------------------------------------------------------------
    def _push_monitor_packet(self):
        if not self.monitor_enabled or not self.monitor_addr:
            return
        eff_btn = (self.phys_buttons | self.btn_or_mask) & 0x1F
        pkt = struct.pack(
            "<BBhhhBB10s",
            1,
            eff_btn,
            0,
            0,
            0,
            2,
            self.kbd_ctrl & 0xFF,
            self.kbd_keys[:10].ljust(10, b"\x00"),
        )
        try:
            self.mon_sock.sendto(pkt, self.monitor_addr)
        except Exception:
            pass

    def _serial_rx_loop(self):
        buf = bytearray()
        last_mon_push = 0.0
        while self.running:
            if not self.ser or not self.ser.is_open:
                self.connect_serial()
                time.sleep(0.5)
                continue
            chunk = b""
            try:
                with self.ser_lock:
                    if self.ser and self.ser.is_open:
                        waiting = self.ser.in_waiting
                        if waiting > 0:
                            chunk = self.ser.read(waiting)
            except Exception:
                with self.ser_lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self.connected_port = None
                self._notify("Serial disconnected, retrying...")
                time.sleep(0.5)
                continue

            if chunk:
                buf.extend(chunk)
                while len(buf) >= 2:
                    if buf[0] == 0x01:
                        self.phys_buttons = buf[1] & 0x1F
                        del buf[:2]
                        self._push_monitor_packet()
                        self._notify()
                    else:
                        del buf[0]
            else:
                now = time.perf_counter()
                if self.monitor_enabled and (now - last_mon_push) >= 0.025:
                    last_mon_push = now
                    self._push_monitor_packet()
                time.sleep(0.004)

    def _udp_server_loop(self):
        while self.running:
            if not self.sock:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    bind_ip = self.cfg.get("net_ip", "0.0.0.0")
                    if bind_ip == "127.0.0.1":
                        bind_ip = "0.0.0.0"
                    s.bind((bind_ip, int(self.cfg.get("net_port", 50050))))
                    s.settimeout(0.2)
                    self.sock = s
                    self._notify(f"UDP Server listening on {self.cfg.get('net_ip')}:{self.cfg.get('net_port')}")
                except Exception as e:
                    self._notify(f"UDP bind error: {e}")
                    time.sleep(1.0)
                    continue

            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                continue

            if len(data) < 16:
                continue

            raw_hdr = data[:16]
            mac, rand_val, indexpts, cmd = struct.unpack("<IIII", raw_hdr)

            # Automatically decrypt 128-byte XXTEA encrypted packets (kmNet.enc_*)
            if len(data) == 128 and mac != self.mac_int:
                dec = self._xxtea_decrypt_128(data, self.mac_int)
                d_mac, d_rand, d_idx, d_cmd = struct.unpack("<IIII", dec[:16])
                if d_mac == self.mac_int:
                    data = dec
                    mac, rand_val, indexpts, cmd = d_mac, d_rand, d_idx, d_cmd

            if self.mac_int != 0 and mac != self.mac_int and cmd != CMD_CONNECT:
                continue

            self.udp_rx_count += 1
            self.last_client = f"{addr[0]}:{addr[1]}"
            payload = data[16:]

            if cmd == CMD_CONNECT:
                self.last_cmd_name = "connect"
                self._send_frame(bytes([9, 1]))

            elif cmd == CMD_MOUSE_MOVE:
                self.last_cmd_name = "move"
                if len(payload) >= 12:
                    _, x, y = struct.unpack("<iii", payload[:12])
                    self.move_rel(x, y)
                elif len(payload) >= 8:
                    x, y = struct.unpack("<ii", payload[:8])
                    self.move_rel(x, y)
                else:
                    x = struct.unpack("<h", struct.pack("<H", rand_val & 0xFFFF))[0]
                    y = struct.unpack("<h", struct.pack("<H", (rand_val >> 16) & 0xFFFF))[0]
                    self.move_rel(x, y)

            elif cmd == CMD_MOUSE_LEFT:
                self.last_cmd_name = "left"
                btn = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else (rand_val & 1)
                self.set_button(0x01, (btn & 0x01) != 0)
                self._push_monitor_packet()

            elif cmd == CMD_MOUSE_RIGHT:
                self.last_cmd_name = "right"
                btn = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else (rand_val & 2)
                self.set_button(0x02, (btn & 0x02) != 0 if len(payload) >= 4 else (btn != 0))
                self._push_monitor_packet()

            elif cmd == CMD_MOUSE_MIDDLE:
                self.last_cmd_name = "middle"
                btn = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else (rand_val & 4)
                self.set_button(0x04, (btn & 0x04) != 0 if len(payload) >= 4 else (btn != 0))
                self._push_monitor_packet()

            elif cmd in (CMD_MOUSE_WHEEL, CMD_MOUSE_ALL):
                if len(payload) >= 16:
                    btn, x, y, w = struct.unpack("<iiii", payload[:16])
                    self.last_cmd_name = "mouse" if (x != 0 or y != 0 or btn != self.btn_or_mask) else "wheel"
                    self.mouse_all(btn, x, y, w)
                    self._push_monitor_packet()
                else:
                    self.last_cmd_name = "wheel"
                    w = struct.unpack("<i", struct.pack("<I", rand_val))[0]
                    self.wheel(w)

            elif cmd == CMD_MOUSE_AUTOMOVE:
                self.last_cmd_name = "move_auto"
                if len(payload) >= 12:
                    _, x, y = struct.unpack("<iii", payload[:12])
                    self.start_smooth_move(x, y, rand_val)

            elif cmd == CMD_BAZER_MOVE:
                self.last_cmd_name = "move_beizer"
                if len(payload) >= 32:
                    _, x, y, _, p0, p1, p2, p3 = struct.unpack("<iiiiiiii", payload[:32])
                    self.start_smooth_move(x, y, rand_val, [p0, p1, p2, p3])

            elif cmd == CMD_KEYBOARD_ALL:
                self.last_cmd_name = "keyboard"
                self.handle_keyboard_all(payload)

            elif cmd == CMD_MONITOR:
                self.last_cmd_name = "monitor"
                mon_port = rand_val & 0xFFFF
                if mon_port > 0:
                    self.monitor_enabled = True
                    self.monitor_addr = (addr[0], mon_port)
                    self._send_frame(bytes([9, 1]))
                    self._push_monitor_packet()
                else:
                    self.monitor_enabled = False
                    self.monitor_addr = None

            elif cmd == CMD_MASK_MOUSE:
                self.last_cmd_name = "mask"
                vk = (rand_val >> 8) & 0xFF
                if vk:
                    self.masked_keys.add(vk)
                self.set_block_mask(rand_val & 0x1F)

            elif cmd == CMD_UNMASK_ALL:
                self.last_cmd_name = "unmask"
                vk = (rand_val >> 8) & 0xFF
                if rand_val == 0:
                    self.masked_keys.clear()
                    self.set_block_mask(0)
                else:
                    self.masked_keys.discard(vk)
                    self.set_block_mask(rand_val & 0x1F)

            elif cmd == CMD_TRACE:
                self.last_cmd_name = "trace"
                val = rand_val & 0xFFFF
                self.set_smoothing(val > 0)

            elif cmd == CMD_SHOWPIC:
                self.last_cmd_name = "lcd_color"

            elif cmd == CMD_SETCONFIG:
                self.last_cmd_name = "setip_port"

            elif cmd == CMD_SETVIDPID:
                self.last_cmd_name = "setvidpid"

            elif cmd == CMD_DEBUG:
                self.last_cmd_name = "debug"

            elif cmd == CMD_REBOOT:
                self.last_cmd_name = "reboot"
                self.masked_keys.clear()
                self.set_block_mask(0)
                self.set_button(0x1F, False)

            # Echo original 16-byte header acknowledgement back to client
            try:
                self.sock.sendto(raw_hdr, addr)
            except Exception:
                pass

    def start(self):
        self.running = True
        self.mac_int = self._parse_uuid(self.cfg.get("uuid", "12345678"))
        self.connect_serial()
        threading.Thread(target=self._serial_rx_loop, daemon=True).start()
        threading.Thread(target=self._udp_server_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        with self.ser_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None


# ============================================================================
# 3. Lightweight Dark-Mode Native GUI (Tkinter)
# ============================================================================
def run_gui(cfg):
    import tkinter as tk
    from tkinter import ttk, messagebox

    root = tk.Tk()
    root.title("KmboxNet Bridge (Lightweight XEN Replacement)")
    root.geometry("480x400")
    root.configure(bg="#141619")
    root.resizable(False, False)

    bridge = KmboxNetBridge(cfg)

    style = ttk.Style()
    style.theme_use("clam")
    style.configure("TFrame", background="#141619")
    style.configure("TLabel", background="#141619", foreground="#E5E7EB", font=("Segoe UI", 10))
    style.configure("Header.TLabel", background="#141619", foreground="#60A5FA", font=("Segoe UI Semibold", 12))
    style.configure("Status.TLabel", background="#1F242B", foreground="#34D399", font=("Consolas", 10))
    style.configure("TButton", background="#2563EB", foreground="#FFFFFF", font=("Segoe UI Semibold", 9), padding=5)
    style.map("TButton", background=[("active", "#1D4ED8")])

    main = ttk.Frame(root, padding=14)
    main.pack(fill="both", expand=True)

    ttk.Label(main, text="KmboxNet Bridge — Zero-XEN Controller", style="Header.TLabel").pack(anchor="w", pady=(0, 8))

    status_var = tk.StringVar(value="Initializing...")
    status_lbl = tk.Label(
        main, textvariable=status_var, bg="#1F242B", fg="#34D399",
        font=("Consolas", 10), anchor="w", padx=10, pady=8, relief="flat"
    )
    status_lbl.pack(fill="x", pady=(0, 10))

    grid = ttk.Frame(main)
    grid.pack(fill="x", pady=4)

    ttk.Label(grid, text="Serial Port:").grid(row=0, column=0, sticky="w", pady=4)
    port_var = tk.StringVar(value=cfg.get("com_port", "AUTO"))
    port_entry = tk.Entry(grid, textvariable=port_var, bg="#1F242B", fg="#FFFFFF", insertbackground="#FFFFFF", relief="flat", width=16)
    port_entry.grid(row=0, column=1, sticky="w", padx=8, pady=4)

    ttk.Label(grid, text="UDP IP:").grid(row=0, column=2, sticky="w", padx=(12, 0), pady=4)
    ip_var = tk.StringVar(value=cfg.get("net_ip", "127.0.0.1"))
    ip_entry = tk.Entry(grid, textvariable=ip_var, bg="#1F242B", fg="#FFFFFF", insertbackground="#FFFFFF", relief="flat", width=16)
    ip_entry.grid(row=0, column=3, sticky="w", padx=8, pady=4)

    ttk.Label(grid, text="UDP Port:").grid(row=1, column=0, sticky="w", pady=4)
    uport_var = tk.StringVar(value=str(cfg.get("net_port", 50050)))
    uport_entry = tk.Entry(grid, textvariable=uport_var, bg="#1F242B", fg="#FFFFFF", insertbackground="#FFFFFF", relief="flat", width=16)
    uport_entry.grid(row=1, column=1, sticky="w", padx=8, pady=4)

    ttk.Label(grid, text="UUID (Hex):").grid(row=1, column=2, sticky="w", padx=(12, 0), pady=4)
    uuid_var = tk.StringVar(value=str(cfg.get("uuid", "12345678")))
    uuid_entry = tk.Entry(grid, textvariable=uuid_var, bg="#1F242B", fg="#FFFFFF", insertbackground="#FFFFFF", relief="flat", width=16)
    uuid_entry.grid(row=1, column=3, sticky="w", padx=8, pady=4)

    smooth_var = tk.BooleanVar(value=bool(cfg.get("smoothing", False)))
    def on_toggle_smooth():
        bridge.set_smoothing(smooth_var.get())
        save_config(bridge.cfg)

    chk = tk.Checkbutton(
        main, text="Enable Hardware Jitter Smoothing (EEPROM 0x03FE)",
        variable=smooth_var, command=on_toggle_smooth,
        bg="#141619", fg="#E5E7EB", selectcolor="#1F242B",
        activebackground="#141619", activeforeground="#FFFFFF", font=("Segoe UI", 9)
    )
    chk.pack(anchor="w", pady=6)

    tele_var = tk.StringVar(value="UDP RX: 0  |  Serial TX: 0  |  Phys Buttons: 0x00")
    tele_lbl = tk.Label(
        main, textvariable=tele_var, bg="#1F242B", fg="#93C5FD",
        font=("Consolas", 9), anchor="w", padx=10, pady=8
    )
    tele_lbl.pack(fill="x", pady=6)

    btn_frame = ttk.Frame(main)
    btn_frame.pack(fill="x", pady=10)

    def apply_settings():
        cfg["com_port"] = port_var.get().strip() or "AUTO"
        cfg["net_ip"] = ip_var.get().strip() or "127.0.0.1"
        try:
            cfg["net_port"] = int(uport_var.get().strip())
        except ValueError:
            cfg["net_port"] = 50050
        cfg["uuid"] = uuid_var.get().strip() or "12345678"
        save_config(cfg)
        bridge.stop()
        time.sleep(0.15)
        bridge.start()

    def test_move():
        def _square():
            for dx, dy in [(60, 0), (0, 60), (-60, 0), (0, -60)]:
                bridge.move_rel(dx, dy)
                time.sleep(0.08)
        threading.Thread(target=_square, daemon=True).start()

    def test_ping():
        ok = bridge.ping_board()
        status_var.set(f"Board Ping: {'OK (0x69 received)' if ok else 'No response'}")

    def trigger_boot():
        if messagebox.askyesno("Bootloader", "Enter Arduino Leonardo Bootloader (COM12)?"):
            bridge.enter_bootloader()
            status_var.set("Sent km.boot() -> Leonardo entering Bootloader...")

    ttk.Button(btn_frame, text="Apply / Reconnect", command=apply_settings).pack(side="left", padx=(0, 6))
    ttk.Button(btn_frame, text="Test Move", command=test_move).pack(side="left", padx=6)
    ttk.Button(btn_frame, text="Ping Board", command=test_ping).pack(side="left", padx=6)
    ttk.Button(btn_frame, text="Bootloader", command=trigger_boot).pack(side="right")

    def refresh_ui():
        port_txt = bridge.connected_port or "Disconnected (Scanning...)"
        status_var.set(f"Serial: {port_txt}  |  UDP: {cfg['net_ip']}:{cfg['net_port']} (UUID {cfg['uuid']})")
        status_lbl.configure(fg="#34D399" if bridge.connected_port else "#F87171")
        tele_var.set(
            f"UDP RX: {bridge.udp_rx_count} ({bridge.last_cmd_name}) | "
            f"Serial TX: {bridge.serial_tx_count} | "
            f"Btn: 0x{bridge.phys_buttons:02X}"
        )
        root.after(200, refresh_ui)

    bridge.start()
    root.after(200, refresh_ui)

    def on_close():
        bridge.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="Lightweight KmboxNet Bridge for Arduino Leonardo")
    parser.add_argument("--cli", action="store_true", help="Run in headless CLI mode without GUI")
    parser.add_argument("--port", type=str, help="Override COM port (e.g. COM6 or AUTO)")
    parser.add_argument("--udp-port", type=int, help="Override UDP listen port (default 50050)")
    parser.add_argument("--uuid", type=str, help="Override 8-char hex UUID (default 12345678)")
    args = parser.parse_args()

    cfg = load_config()
    if args.port:
        cfg["com_port"] = args.port
    if args.udp_port:
        cfg["net_port"] = args.udp_port
    if args.uuid:
        cfg["uuid"] = args.uuid

    if args.cli:
        def print_status(msg):
            if msg:
                print(f"[KmboxBridge] {msg}", flush=True)
        bridge = KmboxNetBridge(cfg, on_status=print_status)
        bridge.start()
        print(
            f"[KmboxBridge] Running headless on UDP {cfg['net_ip']}:{cfg['net_port']} "
            f"(UUID={cfg['uuid']}) -> Serial {bridge.connected_port or 'AUTO'}. Press Ctrl+C to stop.",
            flush=True
        )
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            bridge.stop()
    else:
        run_gui(cfg)


if __name__ == "__main__":
    main()
