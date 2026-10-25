# Xeno Mouse

**Xeno Mouse** 是一套專為 **Arduino Leonardo (`ATmega32U4`) + USB Host Shield 2.0 (`MAX3421E`)** 打造的通用型低延遲滑鼠直通韌體與輕量級 **Kmbox Net (UDP) 橋接控制器**。

本專案解決了高階電競滑鼠（如 Razer Viper Mini 等超過 128 bytes 複合式 HID Report Descriptor 滑鼠）與平價低速光學滑鼠在標準 USB Host Shield 上無法移動、枚舉失敗或斷線死鎖的問題，並提供 100% 相容官方 `kmNet` 全部 46 個指令（含 128-byte XXTEA 加密封包）的輕量橋接服務，完全不需依賴龐大的第三方上位機軟體。

---

## 核心特色

1. **位元級動態 HID Report Descriptor 解析器（Bit-Level HID Parser）**
   - 自動解析任意滑鼠的 `Buttons`、`X`、`Y`、`Wheel` 位元偏移與寬度（支援 8-bit / 12-bit / 16-bit 座標與多介面複合裝置），不再受限於 Boot Protocol 或 128-byte 描述子截斷。
2. **MAX3421E 主動匯流排自癒探測（Active Bus Probing & Self-Healing）**
   - 解決 `USB_Host_Shield_2.0` 僅依賴 `bmCONDETIRQ` 邊緣中斷導致接觸彈跳（Contact Bounce）或重啟後卡死在 `WAIT_FOR_DEVICE` 的問題。
   - 內建 `bmSAMPLEBUS` 主動採樣與全階段 3 次控制傳輸重試（`getDevDescr`、`setAddr`、`getConfDescr`、`setConf`）。
3. **純 HID 隱藏 COM Port 模式（`-DCDC_DISABLED`）**
   - 編譯時徹底移除 USB CDC 虛擬序列埠描述子與端點，Windows 裝置管理員僅識別為標準純 USB HID 滑鼠。
4. **輕量級 Zero-Dependency `KmboxNetBridge`（GUI + CLI）**
   - 完整相容官方 `kmNet.pyd` **全部 46 個函式**：
     - 明文與 128-byte XXTEA 加密指令：`move` / `enc_move`、`left` / `right` / `middle` / `wheel` / `mouse` 及對應 `enc_*`
     - 硬體級 500Hz 軌跡插值：`move_auto`、`enc_move_auto`、`move_beizer`、`enc_move_beizer`
     - 實體按鍵即時監聽：`monitor`、`isdown_left`、`isdown_right`、`isdown_middle`、`isdown_side1`、`isdown_side2`、`isdown_keyboard`
     - 實體按鍵攔截遮罩：`mask_left`、`mask_right`、`mask_middle`、`mask_side1`、`mask_side2`、`unmask_all`
5. **雙協議原生支援（`Serial1` @ `115200` baud）**
   - 同時支援原機 XEN 二進位協議（`0x14`/`0x15` 挑戰回應握手 + 17 組控制指令）與標準 ASCII `km.*` 指令（含 `km.boot()` 免按鍵遠端進入 Bootloader）。

---

## 專案結構

```text
xeno-mouse/
├── firmware/
│   └── universal_passthrough/
│       └── universal_passthrough.ino   # Arduino Leonardo + USB Host Shield 通用自癒直通韌體
├── bridge/
│   ├── kmbox_bridge.py                 # 輕量級 Kmbox Net (UDP) <-> Serial1 橋接器 (含 GUI 與 CLI)
│   └── requirements.txt                # Python 依賴套件 (pyserial)
├── examples/
│   └── test_kmnet_client.py            # Kmbox Net UDP 客戶端連線與控制測試範例
└── LICENSE                             # MIT License
```

---

## 硬體接線

| Arduino Leonardo (`Serial1`) | 外部 USB-TTL 模組（如 CH340） |
| :--- | :--- |
| **Pin 0 (`RX1`)** | **TXD** |
| **Pin 1 (`TX1`)** | **RXD** |
| **GND** | **GND** |

- **USB Host Shield 2.0**：堆疊於 Arduino Leonardo 上（確保 `VBUS 5V` 與 `3.3V` 供電跳線已焊接），外接實體 USB 滑鼠。
- **Arduino Leonardo Micro-USB**：連接至電腦（識別為純 USB HID 滑鼠）。

---

## 編譯與燒錄

使用 `arduino-cli` 搭配 `-DCDC_DISABLED` 編譯（隱藏虛擬 COM Port）：

```bash
arduino-cli compile --fqbn arduino:avr:leonardo \
  --build-property "compiler.c.extra_flags=-DCDC_DISABLED" \
  --build-property "compiler.cpp.extra_flags=-DCDC_DISABLED" \
  --output-dir ./build ./firmware/universal_passthrough
```

---

## 啟動 KmboxNet Bridge

### 1. 安裝依賴
```bash
pip install -r bridge/requirements.txt
```

### 2. 啟動深色圖形控制面板（GUI）
```bash
python bridge/kmbox_bridge.py
```

### 3. 啟動無介面背景服務（CLI）
```bash
python bridge/kmbox_bridge.py --cli --port AUTO --udp-port 50050 --uuid 12345678
```

---

## License

本專案採用 [MIT License](./LICENSE) 授權。
