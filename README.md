# Xeno Mouse

**Xeno Mouse** 是一套專為 **Arduino Leonardo (`ATmega32U4`) + USB Host Shield 2.0 (`MAX3421E`)** 打造的通用型低延遲滑鼠直通韌體，同時支援 **MAKCU 官方硬體 API** 與 **Kmbox Net (UDP)** 雙通訊協議。

本專案解決了高階電競滑鼠（如 Razer Viper Mini 等超過 128 bytes 複合式 HID Report Descriptor 滑鼠）與平價低速光學滑鼠在標準 USB Host Shield 上無法移動、枚舉失敗或斷線死鎖的問題，並提供原生相容 [MAKCU API](https://makcu.com/en/api/) 與官方 `kmNet.pyd` 全部 46 個指令（含 128-byte XXTEA 加密封包）的控制介面。

---

## 核心特色

1. **全面支援 [MAKCU 官方 API 格式](https://makcu.com/en/api/)**
   - **MAKCU ASCII 協議**：支援 `.cmd(...)` 與 `km.cmd(...)` 語法（如 `.move(x,y)`、`km.move(x,y)`、`km.left(1)`、`km.click(1)`、`km.mo(...)`、`km.version()`、`km.baud(rate)`、`km.echo(0/1)`、`km.init()`、`km.unmask_all()`）。
   - **MAKCU 7-Byte 極速二進位協議 (`0xA5`)**：支援固定 7-byte CRC8 緊湊封包 `[0xA5][dx:i16 LE][dy:i16 LE][buttons:u8][crc8:u8]`（多項式 0x07），專供射擊輔助／高回報率算法以微秒級延遲直接傳輸。
   - **MAKCU V2 二進位封包 (`0x50`)**：支援 `[0x50][CMD][LEN_LO][LEN_HI][PAYLOAD...]` 完整幀結構。
   - **MAKCU V4 封包 (`DE AD`)**：支援動態鮑率無縫切換（`DE AD len 0xA5 baud`）。
   - 提供專屬 Python 客戶端庫 `bridge/makcu.py` 與範例 `examples/test_makcu_client.py`。

2. **位元級動態 HID Report Descriptor 解析器（Bit-Level HID Parser）**
   - 自動解析任意滑鼠的 `Buttons`、`X`、`Y`、`Wheel` 位元偏移與寬度（支援 8-bit / 12-bit / 16-bit 座標與多介面複合裝置），不再受限於 Boot Protocol 或 128-byte 描述子截斷。

3. **MAX3421E 主動匯流排自癒探測（Active Bus Probing & Self-Healing）**
   - 解決 `USB_Host_Shield_2.0` 僅依賴 `bmCONDETIRQ` 邊緣中斷導致接觸不良、接點彈跳（Contact Bounce）或重啟後卡死在 `WAIT_FOR_DEVICE`（`L` 燈固定 500ms 閃爍）的問題。
   - 內建 `bmSAMPLEBUS` + `rHRSL` 主動採樣與全階段 3 次控制傳輸重試（`getDevDescr`、`setAddr`、`getConfDescr`、`setConf`）。

4. **純 HID 隱藏 COM Port 模式（`-DCDC_DISABLED`）**
   - 編譯時徹底移除 USB CDC 虛擬序列埠描述子與端點，Windows 裝置管理員僅識別為標準純 USB HID 滑鼠。

5. **輕量級 Zero-Dependency `KmboxNetBridge`（GUI + CLI）**
   - 完整相容官方 `kmNet.pyd` **全部 46 個函式**：
     - 明文與 128-byte XXTEA 加密指令：`move` / `enc_move`、`left` / `right` / `middle` / `wheel` / `mouse` 及對應 `enc_*`
     - 硬體級 500Hz 軌跡插值：`move_auto`、`enc_move_auto`、`move_beizer`、`enc_move_beizer`
     - 實體按鍵即時監聽：`monitor`、`isdown_left`、`isdown_right`、`isdown_middle`、`isdown_side1`、`isdown_side2`、`isdown_keyboard`
     - 實體按鍵攔截遮罩：`mask_left`、`mask_right`、`mask_middle`、`mask_side1`、`mask_side2`、`unmask_all`

6. **三協議原生並存（`Serial1` @ `115200` baud）**
   - 同時支援 MAKCU 協議、Kmbox Net 協議與原機 XEN 二進位協議（`0x14`/`0x15` 挑戰回應握手 + 17 組控制指令），並具備 `km.boot()` 免按鍵遠端進入 Bootloader 功能。

---

## 專案結構

```text
xeno-mouse/
├── firmware/
│   └── universal_passthrough/
│       └── universal_passthrough.ino   # Arduino Leonardo + USB Host Shield 通用自癒直通韌體
├── bridge/
│   ├── kmbox_bridge.py                 # 輕量級 Kmbox Net (UDP) <-> Serial1 橋接器 (含 GUI 與 CLI)
│   ├── makcu.py                        # MAKCU 官方硬體 API Python 客戶端庫
│   └── requirements.txt                # Python 依賴套件 (pyserial)
├── examples/
│   ├── test_kmnet_client.py            # Kmbox Net UDP 客戶端連線與全指令測試範例
│   └── test_makcu_client.py            # MAKCU 協議（ASCII + V2 二進位 + 7-byte CRC8）測試範例
├── scripts/
│   └── build_and_flash.py              # 自動化編譯與燒錄腳本 (支援 km.boot 遠端重啟)
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

### 一鍵腳本（自動編譯與遠端觸發燒錄）
```bash
python scripts/build_and_flash.py --flash
```

### 手動編譯
使用 `arduino-cli` 搭配 `-DCDC_DISABLED` 與 `-mcall-prologues` 編譯（隱藏虛擬 COM Port 並最佳化容量）：

```bash
arduino-cli compile --fqbn arduino:avr:leonardo \
  --build-property "compiler.c.extra_flags=-DCDC_DISABLED -mcall-prologues" \
  --build-property "compiler.cpp.extra_flags=-DCDC_DISABLED -mcall-prologues" \
  --output-dir ./build ./firmware/universal_passthrough
```

---

## 啟動與使用

### 1. 使用 MAKCU API（Python 客戶端）
```python
from bridge.makcu import Makcu

dev = Makcu("COM6", 115200)

# 1. MAKCU ASCII 移動
dev.move(100, 50)

# 2. MAKCU 點前綴指令
dev.send_cmd(".move(20, 20)")

# 3. MAKCU 7-Byte 高速 CRC8 二進位移動 (專為高效能視覺算法設計)
dev.send_binary_7b(40, 20, 0)

# 4. 按鍵點擊與狀態查詢
dev.click(1)
print(dev.left())  # 查詢左鍵狀態 (0=無, 1=實體, 2=注入, 3=兩者)
```

### 2. 啟動 KmboxNet Bridge 控制面板（GUI）
```bash
python bridge/kmbox_bridge.py
```

### 3. 啟動 KmboxNet Bridge 背景服務（CLI）
```bash
python bridge/kmbox_bridge.py --cli --port AUTO --udp-port 50050 --uuid 12345678
```

---

## License

本專案採用 [MIT License](./LICENSE) 授權。
