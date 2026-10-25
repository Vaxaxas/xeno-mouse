#include <SPI.h>
#include <HID.h>
#include <usbhub.h>
#include <hidcomposite.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/eeprom.h>

struct DeviceCryptoState {
    uint8_t round_keys[4][16]; // 0x0465..0x04A4 (Round 3 starts at 0x0495)
    uint8_t add_keys[4];       // 0x04A5..0x04A8
    uint8_t mul_keys[4];       // 0x04A9..0x04AC
};

// ============================================================================
// 1. Custom 5-Button + X/Y/Wheel HID Mouse Device (Arduino -> PC)
// ============================================================================
static const uint8_t _customMouseReportDescriptor[] PROGMEM = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x01,                    //     REPORT_ID (1)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x05,                    //     USAGE_MAXIMUM (Button 5)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x05,                    //     REPORT_COUNT (5)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x03,                    //     REPORT_SIZE (3)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

static HIDSubDescriptor mouseReportNode(_customMouseReportDescriptor, sizeof(_customMouseReportDescriptor));

class SmoothMouseDevice {
private:
    uint8_t current_buttons = 0;
    uint8_t last_sent_buttons = 0;
    bool buttons_dirty = false;
    int16_t accum_x = 0;
    int16_t accum_y = 0;
    int16_t accum_wheel = 0;

    static inline int8_t clamp127(int16_t v) {
        if (v > 127) return 127;
        if (v < -127) return -127;
        return (int8_t)v;
    }

public:
    SmoothMouseDevice() {
        HID().AppendDescriptor(&mouseReportNode);
    }

    void resetState() {
        if (last_sent_buttons != 0) {
            uint8_t m[4] = {0, 0, 0, 0};
            HID().SendReport(1, m, 4);
        }
        current_buttons = 0;
        last_sent_buttons = 0;
        buttons_dirty = false;
        accum_x = 0;
        accum_y = 0;
        accum_wheel = 0;
    }

    void feedInput(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel) {
        uint8_t masked_btn = buttons & 0x1F;
        if (masked_btn != current_buttons) {
            current_buttons = masked_btn;
            buttons_dirty = true;
        }
        accum_x += dx;
        accum_y += dy;
        accum_wheel += wheel;
    }

    bool flushOnce() {
        if (!buttons_dirty && accum_x == 0 && accum_y == 0 && accum_wheel == 0) {
            return false;
        }
        int8_t sx = clamp127(accum_x);
        int8_t sy = clamp127(accum_y);
        int8_t sw = clamp127(accum_wheel);

        uint8_t m[4];
        m[0] = current_buttons;
        m[1] = (uint8_t)sx;
        m[2] = (uint8_t)sy;
        m[3] = (uint8_t)sw;

        HID().SendReport(1, m, 4);

        accum_x -= sx;
        accum_y -= sy;
        accum_wheel -= sw;
        last_sent_buttons = current_buttons;
        buttons_dirty = false;
        return true;
    }
};

static SmoothMouseDevice PcMouse;

// ============================================================================
// 2. Dynamic Bit-Level HID Report Descriptor Parser
// ============================================================================
struct MouseReportLayout {
    bool valid;
    bool is_fallback;
    bool has_report_id;
    uint8_t report_id;
    uint16_t btn_offset;
    uint8_t btn_size;
    uint16_t x_offset;
    uint8_t x_size;
    uint16_t y_offset;
    uint8_t y_size;
    uint16_t wheel_offset;
    uint8_t wheel_size;

    void clear() {
        valid = false;
        is_fallback = false;
        has_report_id = false;
        report_id = 0;
        btn_offset = 0;
        btn_size = 0;
        x_offset = 0;
        x_size = 0;
        y_offset = 0;
        y_size = 0;
        wheel_offset = 0;
        wheel_size = 0;
    }
};

class HidReportDescParser : public USBReadParser {
private:
    MouseReportLayout &out;

    // Parser byte-stream state for cross-chunk items
    uint8_t item_prefix = 0;
    uint8_t item_size = 0;
    uint8_t item_bytes_read = 0;
    uint32_t item_val = 0;
    bool in_item = false;
    uint8_t skip_long_bytes = 0;

    // HID Global state
    uint16_t usage_page = 0;
    uint8_t report_size = 0;
    uint8_t report_count = 0;
    uint8_t cur_report_id = 0;
    bool seen_report_id = false;
    uint16_t bit_offset = 0;

    // HID Local state
    static const uint8_t MAX_USAGES = 16;
    uint16_t usages[MAX_USAGES];
    uint8_t usage_count = 0;
    uint16_t usage_min = 0;
    uint16_t usage_max = 0;
    bool has_usage_range = false;

    // Candidate tracking for the active Report ID
    bool locked = false;
    bool found_x = false;
    bool found_y = false;
    uint16_t cand_btn_offset = 0;
    uint8_t cand_btn_size = 0;
    uint16_t cand_wheel_offset = 0;
    uint8_t cand_wheel_size = 0;

    void clearLocalState() {
        usage_count = 0;
        usage_min = 0;
        usage_max = 0;
        has_usage_range = false;
    }

    void processItem(uint8_t prefix, uint32_t val) {
        if (locked) return;

        uint8_t type = (prefix >> 2) & 0x03;
        uint8_t tag  = (prefix >> 4) & 0x0F;

        if (type == 1) { // Global Item
            switch (tag) {
                case 0x0: // Usage Page
                    usage_page = (uint16_t)(val & 0xFFFF);
                    break;
                case 0x7: // Report Size
                    report_size = (uint8_t)(val & 0xFF);
                    break;
                case 0x8: { // Report ID
                    uint8_t new_id = (uint8_t)(val & 0xFF);
                    if (found_x && found_y) {
                        locked = true;
                        return;
                    }
                    seen_report_id = true;
                    cur_report_id = new_id;
                    bit_offset = 8; // 8 bits for the Report ID byte itself
                    cand_btn_offset = 0;
                    cand_btn_size = 0;
                    cand_wheel_offset = 0;
                    cand_wheel_size = 0;
                    found_x = false;
                    found_y = false;
                    break;
                }
                case 0x9: // Report Count
                    report_count = (uint8_t)(val & 0xFF);
                    break;
                default:
                    break;
            }
        } else if (type == 2) { // Local Item
            switch (tag) {
                case 0x0: // Usage
                    if (usage_count < MAX_USAGES) {
                        usages[usage_count++] = (uint16_t)(val & 0xFFFF);
                    }
                    break;
                case 0x1: // Usage Minimum
                    usage_min = (uint16_t)(val & 0xFFFF);
                    has_usage_range = true;
                    break;
                case 0x2: // Usage Maximum
                    usage_max = (uint16_t)(val & 0xFFFF);
                    has_usage_range = true;
                    break;
                default:
                    break;
            }
        } else if (type == 0) { // Main Item
            if (tag == 0x8) { // INPUT
                bool is_const = (val & 0x01) != 0;
                uint16_t total_bits = (uint16_t)report_size * (uint16_t)report_count;

                if (!is_const && total_bits > 0) {
                    if (usage_page == 0x09) { // Button Page
                        if (cand_btn_size == 0) {
                            cand_btn_offset = bit_offset;
                            cand_btn_size = (total_bits > 8) ? 8 : (uint8_t)total_bits;
                        } else if (cand_btn_offset + cand_btn_size == bit_offset && cand_btn_size < 8) {
                            uint8_t merged = cand_btn_size + (uint8_t)total_bits;
                            cand_btn_size = (merged > 8) ? 8 : merged;
                        }
                    } else if (usage_page == 0x01) { // Generic Desktop Page
                        for (uint8_t k = 0; k < report_count; k++) {
                            uint16_t u = 0;
                            if (k < usage_count) {
                                u = usages[k];
                            } else if (has_usage_range && (usage_min + k <= usage_max)) {
                                u = usage_min + k;
                            } else if (usage_count > 0) {
                                u = usages[usage_count - 1];
                            }

                            uint16_t fld_offset = bit_offset + (uint16_t)k * report_size;
                            if (u == 0x30 && !found_x && report_size >= 8 && report_size <= 16) {
                                out.x_offset = fld_offset;
                                out.x_size = report_size;
                                found_x = true;
                            } else if (u == 0x31 && !found_y && report_size >= 8 && report_size <= 16) {
                                out.y_offset = fld_offset;
                                out.y_size = report_size;
                                found_y = true;
                            } else if (u == 0x38 && cand_wheel_size == 0 && report_size >= 8 && report_size <= 16) {
                                cand_wheel_offset = fld_offset;
                                cand_wheel_size = report_size;
                            }
                        }
                    }
                }

                bit_offset += total_bits;

                if (found_x && found_y) {
                    out.valid = true;
                    out.has_report_id = seen_report_id;
                    out.report_id = cur_report_id;
                    out.btn_offset = cand_btn_offset;
                    out.btn_size = cand_btn_size;
                    out.wheel_offset = cand_wheel_offset;
                    out.wheel_size = cand_wheel_size;
                }
            }
            clearLocalState();
        }
    }

public:
    HidReportDescParser(MouseReportLayout &target) : out(target) {
        out.clear();
    }

    void Parse(const uint16_t len, const uint8_t *pbuf, const uint16_t &offset __attribute__((unused))) override {
        for (uint16_t i = 0; i < len; i++) {
            uint8_t b = pbuf[i];

            if (skip_long_bytes > 0) {
                if (in_item) {
                    skip_long_bytes = b + 2;
                    in_item = false;
                } else {
                    skip_long_bytes--;
                }
                continue;
            }

            if (!in_item) {
                if (b == 0xFE) {
                    in_item = true;
                    skip_long_bytes = 1;
                    continue;
                }
                item_prefix = b;
                uint8_t sz_code = b & 0x03;
                item_size = (sz_code == 3) ? 4 : sz_code;
                item_bytes_read = 0;
                item_val = 0;
                if (item_size == 0) {
                    processItem(item_prefix, 0);
                } else {
                    in_item = true;
                }
            } else {
                item_val |= ((uint32_t)b << (item_bytes_read * 8));
                item_bytes_read++;
                if (item_bytes_read >= item_size) {
                    in_item = false;
                    processItem(item_prefix, item_val);
                }
            }
        }
    }
};

// ============================================================================
// 3. Bit-field Extraction Helpers
// ============================================================================
#line 361 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t extractUnsignedBits(const uint8_t *buf, uint8_t len, uint16_t bit_offset, uint8_t bit_size);
#line 377 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static int16_t extractSignedBits(const uint8_t *buf, uint8_t len, uint16_t bit_offset, uint8_t bit_size);
#line 403 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t read_sig_byte(uint8_t addr);
#line 419 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static void srandom_lcg(uint32_t seed);
#line 424 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static int32_t random_lcg(void);
#line 436 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t mod_inv256(uint8_t m);
#line 447 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t swap_nibbles(uint8_t v);
#line 451 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t rol_byte(uint8_t v, uint8_t n);
#line 456 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t xen_deobfuscate(uint8_t val);
#line 526 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static void rotate_right_16(uint8_t block[16], uint8_t n);
#line 1083 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint64_t compute_cmd14_hash(const uint8_t *data, uint8_t len);
#line 2174 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
void setup();
#line 2197 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
void loop();
#line 361 "C:\\Users\\fi\\.arduino-tools\\universal_passthrough\\universal_passthrough.ino"
static uint8_t extractUnsignedBits(const uint8_t *buf, uint8_t len, uint16_t bit_offset, uint8_t bit_size) {
    if (bit_size == 0) return 0;
    if (bit_size > 8) bit_size = 8;
    uint16_t byte_idx = bit_offset >> 3;
    uint8_t bit_shift = bit_offset & 0x07;
    if (byte_idx >= len) return 0;

    uint16_t raw = buf[byte_idx];
    if (byte_idx + 1 < len) {
        raw |= ((uint16_t)buf[byte_idx + 1] << 8);
    }
    raw >>= bit_shift;
    uint16_t mask = (1U << bit_size) - 1U;
    return (uint8_t)(raw & mask);
}

static int16_t extractSignedBits(const uint8_t *buf, uint8_t len, uint16_t bit_offset, uint8_t bit_size) {
    if (bit_size == 0 || bit_size > 16) return 0;
    uint16_t byte_idx = bit_offset >> 3;
    uint8_t bit_shift = bit_offset & 0x07;
    if (byte_idx >= len) return 0;

    uint32_t raw = buf[byte_idx];
    if (byte_idx + 1 < len) raw |= ((uint32_t)buf[byte_idx + 1] << 8);
    if (byte_idx + 2 < len) raw |= ((uint32_t)buf[byte_idx + 2] << 16);

    raw >>= bit_shift;
    uint32_t mask = (1UL << bit_size) - 1UL;
    raw &= mask;

    uint32_t sign_bit = 1UL << (bit_size - 1);
    if (raw & sign_bit) {
        raw |= (~0UL) << bit_size;
    }
    return (int16_t)raw;
}

// ============================================================================
// 4. Cryptographic, Nonce, and Cipher Engine (Disassembly 0x1988, 0x4e3e..0x56d0)
// ============================================================================

// Read factory unique device signature row (SPMCSR=0x21, addresses 0x000E..0x0017)
static inline uint8_t read_sig_byte(uint8_t addr) {
    uint8_t result;
    __asm__ __volatile__ (
        "sts %1, %2\n\t"
        "lpm %0, Z\n\t"
        : "=r" (result)
        : "i" (_SFR_MEM_ADDR(SPMCSR)),
          "r" ((uint8_t)(0x21)),
          "z" ((uint16_t)addr)
    );
    return result;
}

// Park-Miller LCG Random Helper (0x6372)
static int32_t s_lcg_seed = 123456788L;

static void srandom_lcg(uint32_t seed) {
    s_lcg_seed = (int32_t)(seed & 0x7FFFFFFFL);
    if (s_lcg_seed == 0) s_lcg_seed = 123456788L;
}

static int32_t random_lcg(void) {
    int32_t hi = s_lcg_seed / 127773L;
    int32_t lo = s_lcg_seed % 127773L;
    int32_t test = 16807L * lo - 2836L * hi;
    if (test <= 0) {
        test += 2147483647L;
    }
    s_lcg_seed = test;
    return test & 0x7FFFFFFFL;
}

// Modular multiplicative inverse modulo 256 for odd m (0x1a1c..0x1a26)
static inline uint8_t mod_inv256(uint8_t m) {
    if ((m & 1) == 0) return 0;
    uint8_t inv = 1;
    uint8_t acc = m;
    while ((acc & 0xFF) != 1) {
        inv++;
        acc = (uint8_t)(acc + m);
    }
    return inv;
}

static inline uint8_t swap_nibbles(uint8_t v) {
    return (uint8_t)((v << 4) | (v >> 4));
}

static inline uint8_t rol_byte(uint8_t v, uint8_t n) {
    return (uint8_t)((v << n) | (v >> (8 - n)));
}

// One-time deobfuscation function (0x41c4..0x4206)
static uint8_t xen_deobfuscate(uint8_t val) {
    val ^= 0x55;
    val = (uint8_t)(val - 0xC1);
    val = swap_nibbles(val);
    val = rol_byte(val, 2);
    val ^= 0xB4;
    val = (uint8_t)(val - 0x19);
    val = (uint8_t)(val * 0xB7);
    val ^= 0xC8;
    val = (uint8_t)(val - 0x2A);
    val = swap_nibbles(val);
    val = rol_byte(val, 1);
    val ^= 0xA6;
    return val;
}

static void __attribute__((noinline)) init_device_keys(DeviceCryptoState *out_state, uint8_t *nonce_out) {
    char hex_str[21];
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t b = read_sig_byte(0x0E + i);
        uint8_t hi = b >> 4;
        uint8_t lo = b & 0x0F;
        hex_str[i * 2 + 0] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
        hex_str[i * 2 + 1] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
    }
    hex_str[20] = '\0';

    // 3. Compute 16-bit weighted checksum
    int16_t checksum = 0;
    for (uint8_t i = 0; i < 20; i++) {
        checksum += (int16_t)((uint8_t)hex_str[i]) * (i + 1);
    }

    // 4. Deterministic permutation
    int16_t r8 = checksum % 727;
    for (uint8_t i = 0; i < 20; i++) {
        int8_t k = (int8_t)(r8 % 20);
        char tmp = hex_str[i];
        hex_str[i] = hex_str[k];
        hex_str[k] = tmp;
        r8 += 13;
    }

    // 5. Derive Round Keys (64 bytes total: 0x0465..0x04A4)
    memcpy(out_state->round_keys[0], hex_str, 16);
    for (uint8_t r = 1; r <= 3; r++) {
        uint8_t first_byte = out_state->round_keys[r - 1][0];
        for (uint8_t j = 0; j < 15; j++) {
            out_state->round_keys[r][j] = out_state->round_keys[r - 1][j + 1] ^ r;
        }
        out_state->round_keys[r][15] = first_byte ^ r;
    }

    // 6. Derive Affine Additive and Multiplier Keys (0x04A5..0x04AC)
    out_state->add_keys[0] = (uint8_t)(hex_str[0] - 0xDB); // 0x04A5
    out_state->add_keys[1] = (uint8_t)(hex_str[1] - 0xB6); // 0x04A6
    out_state->add_keys[2] = (uint8_t)(hex_str[2] - 0x91); // 0x04A7
    out_state->add_keys[3] = (uint8_t)(hex_str[3] + 0x94); // 0x04A8

    out_state->mul_keys[0] = (uint8_t)((hex_str[3] - 0xEF) | 0x01); // 0x04A9
    out_state->mul_keys[1] = (uint8_t)((hex_str[4] - 0xDE) | 0x01); // 0x04AA
    out_state->mul_keys[2] = (uint8_t)((hex_str[5] - 0xCD) | 0x01); // 0x04AB
    out_state->mul_keys[3] = (uint8_t)((hex_str[6] - 0xBC) | 0x01); // 0x04AC

    // 7. Generate 16-byte random nonce at 0x0327..0x0336
    for (uint8_t i = 0; i < 16; i++) {
        nonce_out[i] = (uint8_t)((random_lcg() % 126) + 1);
    }
}

static void rotate_right_16(uint8_t block[16], uint8_t n) {
    uint8_t tmp[16];
    n %= 16;
    if (n == 0) return;
    memcpy(tmp, block + 16 - n, n);
    memmove(block + n, block, 16 - n);
    memcpy(block, tmp, n);
}

// 0x1988: Decrypt 16-byte block in place using subkeys
static void __attribute__((noinline)) block_decrypt_1988(uint8_t block[16], const DeviceCryptoState *state) {
    for (int r = 4; r >= 1; r--) {
        rotate_right_16(block, (uint8_t)r);
        const uint8_t *sub_key = state->round_keys[r - 1];
        uint8_t add_key = state->add_keys[r - 1];
        uint8_t mul_key = state->mul_keys[r - 1];
        uint8_t inv = mod_inv256(mul_key);

        for (int i = 0; i < 16; i++) {
            uint8_t val = (uint8_t)((inv * block[i]) - add_key);
            val ^= (uint8_t)(sub_key[i] ^ add_key);
            block[i] = val;
        }
    }
}

// 0x50f0..0x5516: Bit-accurate non-linear nonce-to-stack key derivation
static void __attribute__((noinline)) handshake_derive_nonce_bytes(const uint8_t nonce[16], uint8_t stack_16[16]) {
    uint8_t r[32] = {0};
    uint8_t C = 0, T = 0, Z = 0;
    uint8_t ram[256] = {0};
    uint16_t Y = 0x0800;
    r[28] = Y & 0xFF;
    r[29] = (Y >> 8) & 0xFF;

    r[16] = nonce[0];
    r[30] = 85;
    r[6] = r[16] ^ r[30];
    r[17] = nonce[1];
    { uint16_t res = (uint16_t)r[17] + r[6]; C = (res > 0xFF); r[17] = (uint8_t)res; Z = (r[17] == 0); }
    r[21] = 90;
    r[17] ^= r[21]; Z = (r[17] == 0);

    for (int i = 2; i <= 9; i++) {
        r[24] = nonce[i];
        { uint16_t res = (uint16_t)r[17] + r[24]; C = (res > 0xFF); r[17] = (uint8_t)res; Z = (r[17] == 0); }
        if (i < 9) {
            r[17] ^= r[21]; Z = (r[17] == 0);
        }
    }

    r[23] = r[17];
    r[23] ^= r[21]; Z = (r[23] == 0);
    r[24] = nonce[10];
    r[26] = 165;
    r[11] = r[26];
    r[17] ^= r[11]; Z = (r[17] == 0);
    r[22] = r[17];
    r[22] |= r[23]; Z = (r[22] == 0);
    r[22] &= r[24]; Z = (r[22] == 0);
    r[27] = 170;
    r[22] ^= r[27]; Z = (r[22] == 0);
    r[14] = r[22];
    r[15] = r[1];
    r[12] = r[14]; r[13] = r[15];
    r[24] = 3;

L_0x5162:
    { uint16_t res = (uint16_t)r[12] + r[12]; C = (res > 0xFF); r[12] = (uint8_t)res; Z = (r[12] == 0); }
    { uint16_t res = (uint16_t)r[13] + r[13] + C; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    r[24]--; Z = (r[24] == 0);
    if (!Z) goto L_0x5162;

    { uint8_t b7 = r[15] & 0x80; C = r[15] & 1; r[15] = (r[15] >> 1) | b7; Z = (r[15] == 0); }
    { uint8_t old_c = C; C = r[14] & 1; r[14] = (r[14] >> 1) | (old_c << 7); Z = (r[14] == 0); }
    { uint8_t b7 = r[15] & 0x80; C = r[15] & 1; r[15] = (r[15] >> 1) | b7; Z = (r[15] == 0); }
    { uint8_t old_c = C; C = r[14] & 1; r[14] = (r[14] >> 1) | (old_c << 7); Z = (r[14] == 0); }
    r[3] = r[12];
    r[3] ^= r[14]; Z = (r[3] == 0);
    r[31] = r[3];
    T = (r[31] >> 0) & 1;
    { uint8_t old_c = C; C = r[31] & 1; r[31] = (r[31] >> 1) | (old_c << 7); Z = (r[31] == 0); }
    if (T) r[31] |= (1 << 7); else r[31] &= ~(1 << 7);
    r[24] = nonce[13];
    r[26] = r[31];
    { int16_t res = (int16_t)r[26] - r[24]; C = (res < 0); r[26] = (uint8_t)res; Z = (r[26] == 0); }
    r[19] = 60;
    r[26] ^= r[19]; Z = (r[26] == 0);
    r[19] = nonce[14];
    { uint16_t p = (uint16_t)r[26] * r[19]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[19] = r[0];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[18] = 240;
    r[15] = r[18];
    r[19] ^= r[15]; Z = (r[19] == 0);
    r[24] = nonce[15];
    r[24] |= r[19]; Z = (r[24] == 0);
    r[20] = 15;
    r[14] = r[24];
    r[14] ^= r[20]; Z = (r[14] == 0);
    r[24] ^= r[15]; Z = (r[24] == 0);
    r[18] = r[16];
    r[18] ^= r[27]; Z = (r[18] == 0);
    r[18] &= r[24]; Z = (r[18] == 0);
    r[17] &= r[6]; Z = (r[17] == 0);
    r[18] |= r[17]; Z = (r[18] == 0);
    r[17] = r[6];
    r[17] ^= r[24]; Z = (r[17] == 0);
    r[17] &= r[23]; Z = (r[17] == 0);
    r[18] |= r[17]; Z = (r[18] == 0);
    r[25] = 4;
    { uint16_t p = (uint16_t)r[18] * r[25]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[16] = r[0]; r[17] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[16] ^= r[18]; Z = (r[16] == 0);
    r[16] &= 204; Z = (r[16] == 0);
    r[4] = r[16];
    r[17] = 0;
    r[25] = 3;

L_0x51cc:
    { uint8_t b7 = r[17] & 0x80; C = r[17] & 1; r[17] = (r[17] >> 1) | b7; Z = (r[17] == 0); }
    { uint8_t old_c = C; C = r[16] & 1; r[16] = (r[16] >> 1) | (old_c << 7); Z = (r[16] == 0); }
    r[25]--; Z = (r[25] == 0);
    if (!Z) goto L_0x51cc;

    r[16] ^= r[4]; Z = (r[16] == 0);
    r[17] = r[3];
    r[17] &= 51; Z = (r[17] == 0);
    r[5] = r[16];
    r[5] |= r[17]; Z = (r[5] == 0);
    r[16] = r[5];
    r[17] = 0;
    { int16_t res = (int16_t)r[28] - 30; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[((r[28] | (r[29] << 8)) + 1) & 0xFF] = r[17];
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[16];
    { int16_t res = (int16_t)r[28] - 226; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[12] = r[16]; r[13] = r[17];
    { uint16_t res = (uint16_t)r[12] + r[12]; C = (res > 0xFF); r[12] = (uint8_t)res; Z = (r[12] == 0); }
    { uint16_t res = (uint16_t)r[13] + r[13] + C; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    r[25] = 4;

L_0x51f6:
    { uint8_t b7 = r[17] & 0x80; C = r[17] & 1; r[17] = (r[17] >> 1) | b7; Z = (r[17] == 0); }
    { uint8_t old_c = C; C = r[16] & 1; r[16] = (r[16] >> 1) | (old_c << 7); Z = (r[16] == 0); }
    r[25]--; Z = (r[25] == 0);
    if (!Z) goto L_0x51f6;

    r[25] = r[12];
    r[25] ^= r[16]; Z = (r[25] == 0);
    { int16_t res = (int16_t)r[28] - 24; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[25];
    { int16_t res = (int16_t)r[28] - 232; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[25] &= 240; Z = (r[25] == 0);
    { int16_t res = (int16_t)r[28] - 28; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[25];
    { int16_t res = (int16_t)r[28] - 228; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[16] = r[25];
    r[17] = 0;
    { int16_t res = (int16_t)r[28] - 27; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[((r[28] | (r[29] << 8)) + 1) & 0xFF] = r[17];
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[16];
    { int16_t res = (int16_t)r[28] - 229; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint8_t b7 = r[17] & 0x80; C = r[17] & 1; r[17] = (r[17] >> 1) | b7; Z = (r[17] == 0); }
    { uint8_t old_c = C; C = r[16] & 1; r[16] = (r[16] >> 1) | (old_c << 7); Z = (r[16] == 0); }
    { uint8_t b7 = r[17] & 0x80; C = r[17] & 1; r[17] = (r[17] >> 1) | b7; Z = (r[17] == 0); }
    { uint8_t old_c = C; C = r[16] & 1; r[16] = (r[16] >> 1) | (old_c << 7); Z = (r[16] == 0); }
    r[10] = r[30];
    r[10] ^= r[16]; Z = (r[10] == 0);
    r[2] = r[27];
    r[2] ^= r[16]; Z = (r[2] == 0);
    r[2] |= r[19]; Z = (r[2] == 0);
    r[27] = r[19];
    r[27] &= r[10]; Z = (r[27] == 0);
    r[2] ^= r[27]; Z = (r[2] == 0);
    r[27] = r[14];
    r[27] ^= r[2]; Z = (r[27] == 0);
    r[30] = r[24];
    r[30] &= r[2]; Z = (r[30] == 0);
    r[30] |= r[27]; Z = (r[30] == 0);
    r[24] ^= r[18]; Z = (r[24] == 0);
    r[24] ^= r[30]; Z = (r[24] == 0);
    r[24] &= 153; Z = (r[24] == 0);
    r[27] = r[24];
    { uint16_t res = (uint16_t)r[27] + r[27]; C = (res > 0xFF); r[27] = (uint8_t)res; Z = (r[27] == 0); }
    r[27] ^= r[24]; Z = (r[27] == 0);
    r[27] &= 85; Z = (r[27] == 0);
    r[8] = r[27];
    { C = r[8] & 1; r[8] >>= 1; Z = (r[8] == 0); }
    { C = r[8] & 1; r[8] >>= 1; Z = (r[8] == 0); }
    r[8] |= r[27]; Z = (r[8] == 0);
    r[17] = r[5];
    r[17] &= 15; Z = (r[17] == 0);
    r[8] |= r[17]; Z = (r[8] == 0);
    { uint16_t res = (uint16_t)r[8] + r[25]; C = (res > 0xFF); r[8] = (uint8_t)res; Z = (r[8] == 0); }
    r[8] ^= r[11]; Z = (r[8] == 0);
    { int16_t res = (int16_t)r[8] - r[10]; C = (res < 0); r[8] = (uint8_t)res; Z = (r[8] == 0); }
    r[8] ^= r[21]; Z = (r[8] == 0);
    { uint16_t p = (uint16_t)r[2] * r[8]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[7] = r[0];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[25] = 60;
    r[7] ^= r[25]; Z = (r[7] == 0);
    r[11] = r[30];
    r[11] |= r[7]; Z = (r[11] == 0);
    r[25] = r[11];
    r[25] ^= r[20]; Z = (r[25] == 0);
    r[25] &= 170; Z = (r[25] == 0);
    r[9] = r[24];
    r[9] &= r[27]; Z = (r[9] == 0);
    r[17] = r[25];
    r[17] |= r[9]; Z = (r[17] == 0);
    r[27] = ~r[27]; C = 1; Z = (r[27] == 0);
    r[27] &= r[24]; Z = (r[27] == 0);
    r[9] = r[17];
    r[9] |= r[27]; Z = (r[9] == 0);
    r[16] = r[9];
    r[17] = 0;
    r[12] = r[16]; r[13] = r[17];
    { uint16_t res = (uint16_t)r[12] + r[12]; C = (res > 0xFF); r[12] = (uint8_t)res; Z = (r[12] == 0); }
    { uint16_t res = (uint16_t)r[13] + r[13] + C; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    { uint16_t res = (uint16_t)r[12] + r[12]; C = (res > 0xFF); r[12] = (uint8_t)res; Z = (r[12] == 0); }
    { uint16_t res = (uint16_t)r[13] + r[13] + C; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    r[27] = 3;

L_0x52a2:
    { uint8_t b7 = r[17] & 0x80; C = r[17] & 1; r[17] = (r[17] >> 1) | b7; Z = (r[17] == 0); }
    { uint8_t old_c = C; C = r[16] & 1; r[16] = (r[16] >> 1) | (old_c << 7); Z = (r[16] == 0); }
    r[27]--; Z = (r[27] == 0);
    if (!Z) goto L_0x52a2;

    r[27] = r[12];
    r[27] ^= r[16]; Z = (r[27] == 0);
    r[16] = r[27];
    r[16] = (r[16] << 4) | (r[16] >> 4);
    { uint16_t res = (uint16_t)r[16] + r[16]; C = (res > 0xFF); r[16] = (uint8_t)res; Z = (r[16] == 0); }
    { uint16_t res = (uint16_t)r[16] + r[1] + C; C = (res > 0xFF); r[16] = (uint8_t)res; Z = (r[16] == 0); }
    { uint16_t res = (uint16_t)r[16] + r[16]; C = (res > 0xFF); r[16] = (uint8_t)res; Z = (r[16] == 0); }
    { uint16_t res = (uint16_t)r[16] + r[1] + C; C = (res > 0xFF); r[16] = (uint8_t)res; Z = (r[16] == 0); }
    { int16_t res = (int16_t)r[28] - 25; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[16];
    { int16_t res = (int16_t)r[28] - 231; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[13] = r[16];
    { uint16_t res = (uint16_t)r[13] + r[8]; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    r[13] ^= r[20]; Z = (r[13] == 0);
    r[17] = r[13];
    { int16_t res = (int16_t)r[17] - r[7]; C = (res < 0); r[17] = (uint8_t)res; Z = (r[17] == 0); }
    r[7] = r[17];
    r[7] ^= r[15]; Z = (r[7] == 0);
    r[11] ^= r[15]; Z = (r[11] == 0);
    { uint16_t p = (uint16_t)r[11] * r[7]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[20] = r[0];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[17] = 204;
    r[11] = r[17];
    r[20] ^= r[11]; Z = (r[20] == 0);
    r[25] |= r[20]; Z = (r[25] == 0);
    r[17] = 51;
    r[25] ^= r[17]; Z = (r[25] == 0);
    r[17] = r[9];
    r[17] &= r[25]; Z = (r[17] == 0);
    r[12] = r[27];
    r[12] = ~r[12]; C = 1; Z = (r[12] == 0);
    r[12] &= r[17]; Z = (r[12] == 0);
    r[9] ^= r[27]; Z = (r[9] == 0);
    r[9] ^= r[25]; Z = (r[9] == 0);
    r[12] |= r[9]; Z = (r[12] == 0);
    r[16] = 8;
    { uint16_t p = (uint16_t)r[12] * r[16]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[8] = r[0]; r[9] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[16] = r[8];
    r[16] ^= r[12]; Z = (r[16] == 0);
    r[16] &= 15; Z = (r[16] == 0);
    r[8] = r[6];
    r[8] ^= r[16]; Z = (r[8] == 0);
    r[8] |= r[25]; Z = (r[8] == 0);
    r[13] &= r[8]; Z = (r[13] == 0);
    r[8] = r[23];
    r[8] ^= r[16]; Z = (r[8] == 0);
    r[13] ^= r[8]; Z = (r[13] == 0);
    r[17] = 4;
    { uint16_t p = (uint16_t)r[6] * r[17]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[8] = r[0]; r[9] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[17] = r[6];
    r[17] ^= r[8]; Z = (r[17] == 0);
    r[17] &= 136; Z = (r[17] == 0);
    r[13] ^= r[17]; Z = (r[13] == 0);
    { int16_t res = (int16_t)r[28] - 21; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[13];
    { int16_t res = (int16_t)r[28] - 235; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t v = (r[28] | (r[29] << 8)) + 3; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[13];
    { uint16_t v = (r[28] | (r[29] << 8)) - 3; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[9] = r[22];
    r[9] ^= r[12]; Z = (r[9] == 0);
    r[17] = 4;
    { uint16_t p = (uint16_t)r[23] * r[17]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[12] = r[0]; r[13] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[12] ^= r[23]; Z = (r[12] == 0);
    r[23] = r[12];
    r[23] &= 136; Z = (r[23] == 0);
    r[12] = r[23];
    r[12] = ~r[12]; C = 1; Z = (r[12] == 0);
    r[12] ^= r[9]; Z = (r[12] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 4; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[12];
    { uint16_t v = (r[28] | (r[29] << 8)) - 4; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[22] |= r[25]; Z = (r[22] == 0);
    r[23] = 0;
    r[12] = r[22]; r[13] = r[23];
    r[17] = 3;

L_0x5358:
    { uint16_t res = (uint16_t)r[12] + r[12]; C = (res > 0xFF); r[12] = (uint8_t)res; Z = (r[12] == 0); }
    { uint16_t res = (uint16_t)r[13] + r[13] + C; C = (res > 0xFF); r[13] = (uint8_t)res; Z = (r[13] == 0); }
    r[17]--; Z = (r[17] == 0);
    if (!Z) goto L_0x5358;

    { uint8_t b7 = r[23] & 0x80; C = r[23] & 1; r[23] = (r[23] >> 1) | b7; Z = (r[23] == 0); }
    { uint8_t old_c = C; C = r[22] & 1; r[22] = (r[22] >> 1) | (old_c << 7); Z = (r[22] == 0); }
    { uint8_t b7 = r[23] & 0x80; C = r[23] & 1; r[23] = (r[23] >> 1) | b7; Z = (r[23] == 0); }
    { uint8_t old_c = C; C = r[22] & 1; r[22] = (r[22] >> 1) | (old_c << 7); Z = (r[22] == 0); }
    r[22] ^= r[12]; Z = (r[22] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 5; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[22];
    { uint16_t v = (r[28] | (r[29] << 8)) - 5; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[7] ^= r[3]; Z = (r[7] == 0);
    T = (r[7] >> 0) & 1;
    { uint8_t old_c = C; C = r[7] & 1; r[7] = (r[7] >> 1) | (old_c << 7); Z = (r[7] == 0); }
    if (T) r[7] |= (1 << 7); else r[7] &= ~(1 << 7);
    r[7] ^= r[11]; Z = (r[7] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 6; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[7];
    { uint16_t v = (r[28] | (r[29] << 8)) - 6; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[17] = 4;
    { uint16_t p = (uint16_t)r[31] * r[17]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[22] = r[0]; r[23] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[22] ^= r[31]; Z = (r[22] == 0);
    r[22] &= 12; Z = (r[22] == 0);
    r[21] ^= r[22]; Z = (r[21] == 0);
    r[31] ^= r[25]; Z = (r[31] == 0);
    { int16_t res = (int16_t)r[28] - 25; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[22] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    { int16_t res = (int16_t)r[28] - 231; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t res = (uint16_t)r[31] + r[22]; C = (res > 0xFF); r[31] = (uint8_t)res; Z = (r[31] == 0); }
    r[21] ^= r[31]; Z = (r[21] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 7; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[21];
    { uint16_t v = (r[28] | (r[29] << 8)) - 7; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[31] = 4;
    { uint16_t p = (uint16_t)r[26] * r[31]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[22] = r[0]; r[23] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[22] ^= r[26]; Z = (r[22] == 0);
    r[22] &= 12; Z = (r[22] == 0);
    r[22] |= 240; Z = (r[22] == 0);
    { int16_t res = (int16_t)r[26] - r[19]; C = (res < 0); r[26] = (uint8_t)res; Z = (r[26] == 0); }
    r[26] ^= r[22]; Z = (r[26] == 0);
    r[17] = 60;
    r[17] ^= r[26]; Z = (r[17] == 0);
    { int16_t res = (int16_t)r[28] - 25; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[17];
    { int16_t res = (int16_t)r[28] - 231; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t v = (r[28] | (r[29] << 8)) + 8; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[17];
    { uint16_t v = (r[28] | (r[29] << 8)) - 8; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[21] = 4;
    { uint16_t p = (uint16_t)r[19] * r[21]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[22] = r[0]; r[23] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[22] ^= r[19]; Z = (r[22] == 0);
    r[21] = r[22];
    r[21] &= 136; Z = (r[21] == 0);
    { uint16_t p = (uint16_t)r[19] * r[14]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[22] = r[0];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[22] ^= r[15]; Z = (r[22] == 0);
    r[22] ^= r[21]; Z = (r[22] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 9; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[22];
    { uint16_t v = (r[28] | (r[29] << 8)) - 9; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[26] = 4;
    { uint16_t p = (uint16_t)r[14] * r[26]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[22] = r[0]; r[23] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[22] ^= r[14]; Z = (r[22] == 0);
    r[22] &= 136; Z = (r[22] == 0);
    r[22] |= 85; Z = (r[22] == 0);
    r[15] ^= r[22]; Z = (r[15] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 10; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[15];
    { uint16_t v = (r[28] | (r[29] << 8)) - 10; r[28] = v & 0xFF; r[29] = v >> 8; }
    { uint16_t v = (r[28] | (r[29] << 8)) + 11; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[14];
    { uint16_t v = (r[28] | (r[29] << 8)) - 11; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[19] = r[5];
    r[19] = ~r[19]; C = 1; Z = (r[19] == 0);
    r[19] ^= r[18]; Z = (r[19] == 0);
    r[19] &= r[4]; Z = (r[19] == 0);
    r[19] ^= r[18]; Z = (r[19] == 0);
    r[18] ^= r[4]; Z = (r[18] == 0);
    r[18] &= r[5]; Z = (r[18] == 0);
    r[19] |= r[18]; Z = (r[19] == 0);
    r[19] ^= r[11]; Z = (r[19] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 12; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[19];
    { uint16_t v = (r[28] | (r[29] << 8)) - 12; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[31] = 4;
    { uint16_t p = (uint16_t)r[20] * r[31]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[18] = r[0]; r[19] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[20] ^= r[18]; Z = (r[20] == 0);
    r[20] &= 204; Z = (r[20] == 0);
    r[17] = r[4];
    r[17] &= 12; Z = (r[17] == 0);
    r[4] = r[17];
    r[4] ^= r[20]; Z = (r[4] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 13; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[4];
    { uint16_t v = (r[28] | (r[29] << 8)) - 13; r[28] = v & 0xFF; r[29] = v >> 8; }
    { int16_t res = (int16_t)r[28] - 30; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[12] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    r[13] = ram[((r[28] | (r[29] << 8)) + 1) & 0xFF];
    { int16_t res = (int16_t)r[28] - 226; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[18] = 3;

L_0x5444:
    { uint8_t b7 = r[13] & 0x80; C = r[13] & 1; r[13] = (r[13] >> 1) | b7; Z = (r[13] == 0); }
    { uint8_t old_c = C; C = r[12] & 1; r[12] = (r[12] >> 1) | (old_c << 7); Z = (r[12] == 0); }
    r[18]--; Z = (r[18] == 0);
    if (!Z) goto L_0x5444;

    r[5] ^= r[12]; Z = (r[5] == 0);
    { int16_t res = (int16_t)r[28] - 24; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[18] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    { int16_t res = (int16_t)r[28] - 232; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[18] &= 48; Z = (r[18] == 0);
    r[12] = r[18];
    r[12] |= r[5]; Z = (r[12] == 0);
    r[20] = 4;
    { uint16_t p = (uint16_t)r[25] * r[20]; r[0] = p & 0xFF; r[1] = p >> 8; }
    r[18] = r[0]; r[19] = r[1];
    r[1] ^= r[1]; Z = (r[1] == 0);
    r[25] ^= r[18]; Z = (r[25] == 0);
    r[25] &= 12; Z = (r[25] == 0);
    r[25] |= 240; Z = (r[25] == 0);
    r[25] ^= r[12]; Z = (r[25] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 14; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[25];
    { uint16_t v = (r[28] | (r[29] << 8)) - 14; r[28] = v & 0xFF; r[29] = v >> 8; }
    { int16_t res = (int16_t)r[28] - 27; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[18] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    r[19] = ram[((r[28] | (r[29] << 8)) + 1) & 0xFF];
    { int16_t res = (int16_t)r[28] - 229; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t res = (uint16_t)r[18] + r[18]; C = (res > 0xFF); r[18] = (uint8_t)res; Z = (r[18] == 0); }
    { uint16_t res = (uint16_t)r[19] + r[19] + C; C = (res > 0xFF); r[19] = (uint8_t)res; Z = (r[19] == 0); }
    r[18] &= 240; Z = (r[18] == 0);
    { int16_t res = (int16_t)r[28] - 27; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[20] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    r[21] = ram[((r[28] | (r[29] << 8)) + 1) & 0xFF];
    { int16_t res = (int16_t)r[28] - 229; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t res = (uint16_t)r[20] + r[20]; C = (res > 0xFF); r[20] = (uint8_t)res; Z = (r[20] == 0); }
    { uint16_t res = (uint16_t)r[21] + r[21] + C; C = (res > 0xFF); r[21] = (uint8_t)res; Z = (r[21] == 0); }
    { uint16_t res = (uint16_t)r[20] + r[20]; C = (res > 0xFF); r[20] = (uint8_t)res; Z = (r[20] == 0); }
    { uint16_t res = (uint16_t)r[21] + r[21] + C; C = (res > 0xFF); r[21] = (uint8_t)res; Z = (r[21] == 0); }
    { int16_t res = (int16_t)r[28] - 28; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[25] = ram[(r[28] | (r[29] << 8)) & 0xFF];
    { int16_t res = (int16_t)r[28] - 228; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    r[25] ^= r[20]; Z = (r[25] == 0);
    r[25] &= 136; Z = (r[25] == 0);
    r[25] ^= r[18]; Z = (r[25] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 15; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[25];
    { uint16_t v = (r[28] | (r[29] << 8)) - 15; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[18] = r[10];
    r[0] = r[10];
    { uint16_t res = (uint16_t)r[0] + r[0]; C = (res > 0xFF); r[0] = (uint8_t)res; Z = (r[0] == 0); }
    { int16_t res = (int16_t)r[19] - r[19] - C; C = (res < 0); r[19] = (uint8_t)res; Z = (r[19] == 0); }
    r[20] = r[18]; r[21] = r[19];
    { uint16_t res = (uint16_t)r[20] + r[20]; C = (res > 0xFF); r[20] = (uint8_t)res; Z = (r[20] == 0); }
    { uint16_t res = (uint16_t)r[21] + r[21] + C; C = (res > 0xFF); r[21] = (uint8_t)res; Z = (r[21] == 0); }
    { uint16_t res = (uint16_t)r[20] + r[20]; C = (res > 0xFF); r[20] = (uint8_t)res; Z = (r[20] == 0); }
    { uint16_t res = (uint16_t)r[21] + r[21] + C; C = (res > 0xFF); r[21] = (uint8_t)res; Z = (r[21] == 0); }
    r[25] = r[10];
    r[25] ^= r[20]; Z = (r[25] == 0);
    r[25] &= 136; Z = (r[25] == 0);
    r[25] |= 85; Z = (r[25] == 0);
    { uint8_t b7 = r[19] & 0x80; C = r[19] & 1; r[19] = (r[19] >> 1) | b7; Z = (r[19] == 0); }
    { uint8_t old_c = C; C = r[18] & 1; r[18] = (r[18] >> 1) | (old_c << 7); Z = (r[18] == 0); }
    { uint8_t b7 = r[19] & 0x80; C = r[19] & 1; r[19] = (r[19] >> 1) | b7; Z = (r[19] == 0); }
    { uint8_t old_c = C; C = r[18] & 1; r[18] = (r[18] >> 1) | (old_c << 7); Z = (r[18] == 0); }
    r[19] = 21;
    r[18] ^= r[19]; Z = (r[18] == 0);
    r[25] ^= r[18]; Z = (r[25] == 0);
    { int16_t res = (int16_t)r[28] - 28; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 255 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    ram[(r[28] | (r[29] << 8)) & 0xFF] = r[25];
    { int16_t res = (int16_t)r[28] - 228; C = (res < 0); r[28] = (uint8_t)res; Z = (r[28] == 0); }
    { int16_t res = (int16_t)r[29] - 0 - C; C = (res < 0); r[29] = (uint8_t)res; Z = (r[29] == 0); }
    { uint16_t v = (r[28] | (r[29] << 8)) + 16; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[25];
    { uint16_t v = (r[28] | (r[29] << 8)) - 16; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[27] |= r[2]; Z = (r[27] == 0);
    r[2] = r[27];
    r[2] = ~r[2]; C = 1; Z = (r[2] == 0);
    r[2] |= r[30]; Z = (r[2] == 0);
    r[27] &= r[30]; Z = (r[27] == 0);
    r[27] ^= r[2]; Z = (r[27] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 17; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[27];
    { uint16_t v = (r[28] | (r[29] << 8)) - 17; r[28] = v & 0xFF; r[29] = v >> 8; }
    r[24] ^= r[16]; Z = (r[24] == 0);
    r[16] = r[24];
    r[16] = ~r[16]; C = 1; Z = (r[16] == 0);
    r[16] &= r[30]; Z = (r[16] == 0);
    r[30] ^= r[24]; Z = (r[30] == 0);
    r[30] |= r[16]; Z = (r[30] == 0);
    r[11] ^= r[30]; Z = (r[11] == 0);
    { uint16_t v = (r[28] | (r[29] << 8)) + 18; r[28] = v & 0xFF; r[29] = v >> 8; }
    ram[((r[28] | (r[29] << 8)) + 63) & 0xFF] = r[11];
    { uint16_t v = (r[28] | (r[29] << 8)) - 18; r[28] = v & 0xFF; r[29] = v >> 8; }

    for (int i = 0; i < 16; i++) {
        stack_16[i] = ram[(0x0842 + i) & 0xFF];
    }
}

static uint64_t compute_cmd14_hash(const uint8_t *data, uint8_t len) {
    uint64_t h = 0x7F3E9A1C4D2B6F80ULL;
    for (uint8_t i = 0; i < len; i++) {
        h ^= data[i];
        h *= 0x9B2C7D3E1F4A5B67ULL;
    }
    return h;
}

// ============================================================================
// 5. XEN Protocol & kmbox / MAKCU Co-Processor Engine
// ============================================================================
static uint8_t computeCrc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}

class XenKmboxEngine {
public:
    // Core state mirroring disassembly 0x02C1..0x02CA
    int16_t accum_x;              // 0x02C1..0x02C2
    int16_t accum_y;              // 0x02C3..0x02C4
    int8_t  accum_wheel;          // 0x02C5
    uint8_t btn_or_mask;          // 0x02C6
    uint8_t btn_block_mask;       // 0x02C7
    uint8_t base_button_mask;     // 0x026A
    uint8_t smoothing_enabled;    // 0x02C8
    uint8_t btn_report_enabled;   // 0x02CA
    uint8_t last_sent_pc_buttons; // 0x02AC
    uint8_t last_raw_phys_buttons;// 0x02AD

    // Authentication state
    uint8_t authenticated;        // 0x02CB
    uint8_t auth_counter;         // 0x0269
    uint8_t start_byte;           // 0x0267
    uint8_t end_byte;             // 0x0266

    // Crypto state
    DeviceCryptoState crypto_state;
    uint8_t nonce[16];            // 0x0327..0x0336

    // Smoothing state
    uint16_t smooth_interval;     // 0x02B7..0x02B8
    uint32_t next_smooth_time;    // 0x02B9..0x02BC
    uint32_t last_move_time;      // 0x02BD..0x02C0
    int8_t   state_x;             // 0x019D
    uint32_t last_step_time_x;    // 0x019E..0x01A1
    int8_t   state_y;             // 0x0198
    uint32_t last_step_time_y;    // 0x0199..0x019C

    // Timing tracking
    uint32_t last_hid_send_us;

    // Packet reception state
    static const uint8_t RX_BUF_SIZE = 32;
    uint8_t rx_buf[RX_BUF_SIZE];
    uint8_t rx_len;
    bool in_packet;

    // Fallback kmbox ASCII buffer
    static const uint8_t ASCII_BUF_SIZE = 48;
    char ascii_buf[ASCII_BUF_SIZE];
    uint8_t ascii_idx;

public:
    XenKmboxEngine() {
        accum_x = 0;
        accum_y = 0;
        accum_wheel = 0;
        btn_or_mask = 0;
        btn_block_mask = 0;
        base_button_mask = 0;
        smoothing_enabled = 0;
        btn_report_enabled = 0;
        last_sent_pc_buttons = 0;
        last_raw_phys_buttons = 0;
        authenticated = 0;
        auth_counter = 0;
        start_byte = 0x17;
        end_byte = 0x17;
        smooth_interval = 60;
        next_smooth_time = 0;
        last_move_time = 0;
        state_x = 0;
        last_step_time_x = 0;
        state_y = 0;
        last_step_time_y = 0;
        last_hid_send_us = 0;
        rx_len = 0;
        in_packet = false;
        ascii_idx = 0;
    }

    void begin(unsigned long baud = 115200) {
        Serial1.begin(baud);
        Serial1.setTimeout(100);

        // Seed random number generator
        srandom_lcg(micros() ^ ((uint32_t)analogRead(A0) << 16) ^ (uint32_t)analogRead(A1));

        // Derive keys and generate initial nonce
        init_device_keys(&crypto_state, nonce);

        // Initial dynamic delimiters via deobfuscation
        start_byte = xen_deobfuscate(0);
        end_byte = xen_deobfuscate(0);
        base_button_mask = 0;

        // Load smoothing setting from EEPROM (0x03FE)
        uint8_t sm = eeprom_read_byte((const uint8_t *)0x03FE);
        smoothing_enabled = (sm == 0x01) ? 1 : 0;
    }

    // ------------------------------------------------------------------------
    // Physical Mouse Ingestion Pipeline (Disassembly 0x2630..0x2664 & 0x277e)
    // ------------------------------------------------------------------------
    void processPhysicalReport(uint8_t raw_buttons, int16_t dx, int16_t dy, int8_t wheel,
                               uint8_t &out_buttons, int16_t &out_dx, int16_t &out_dy, int8_t &out_wheel) {
        // 1. Button Change Notification (0x264c..0x2660)
        if (btn_report_enabled) {
            if (raw_buttons != last_raw_phys_buttons) {
                uint8_t report_pkt[2] = {0x01, raw_buttons};
                Serial1.write(report_pkt, 2);
            }
        }
        last_raw_phys_buttons = raw_buttons;

        // 2. Button Masking: out = ((raw | OR_mask) & ~BLOCK_mask) | base_modifier
        uint8_t masked = (raw_buttons | btn_or_mask) & (~btn_block_mask & 0x1F);
        out_buttons = (masked | base_button_mask) & 0x1F;

        // 3. Movement Delta Combination
        int16_t comb_x = dx + accum_x;
        int16_t comb_y = dy + accum_y;
        int8_t comb_w = wheel + accum_wheel;

        uint32_t now = millis();
        if (comb_x != 0 || comb_y != 0) {
            last_move_time = now;
        }

        // 4. Apply Smoothing Subsystem (0x10b2..0x12ac) before clearing accum_x/y
        if (smoothing_enabled) {
            applySmoothing(now, comb_x, comb_y);
        }

        accum_x = 0;
        accum_y = 0;
        accum_wheel = 0;

        out_dx = comb_x;
        out_dy = comb_y;
        out_wheel = comb_w;

        last_sent_pc_buttons = out_buttons;
        last_hid_send_us = micros();
    }

    // ------------------------------------------------------------------------
    // Standalone Synthetic Report Evaluation (Disassembly 0x4b24..0x4dce)
    // ------------------------------------------------------------------------
    bool evaluateStandaloneReport(uint8_t &out_buttons, int16_t &out_dx, int16_t &out_dy, int8_t &out_wheel) {
        uint8_t target_buttons = (last_raw_phys_buttons | btn_or_mask) & (~btn_block_mask & 0x1F);
        target_buttons &= 0x1F;

        if (accum_x == 0 && accum_y == 0 && accum_wheel == 0 && target_buttons == last_sent_pc_buttons) {
            return false;
        }

        uint32_t now_us = micros();
        if ((uint32_t)(now_us - last_hid_send_us) < 1000UL) {
            return false;
        }

        int16_t rep_x = accum_x;
        int16_t rep_y = accum_y;
        int8_t rep_w = accum_wheel;

        accum_x = 0;
        accum_y = 0;
        accum_wheel = 0;

        uint32_t now = millis();
        if (rep_x != 0 || rep_y != 0) {
            last_move_time = now;
        }

        if (smoothing_enabled) {
            applySmoothing(now, rep_x, rep_y);
        }

        out_buttons = target_buttons;
        out_dx = rep_x;
        out_dy = rep_y;
        out_wheel = rep_w;

        last_sent_pc_buttons = out_buttons;
        last_hid_send_us = now_us;
        return true;
    }

    // ------------------------------------------------------------------------
    // Serial1 Dispatcher (Binary Framing, Handshake 0x14/0x15, ASCII Fallback)
    // ------------------------------------------------------------------------
    void task() {
        uint32_t now = millis();

        // Dynamic Jitter Interval Randomization (0x4a02..0x4b24)
        if (smoothing_enabled && now >= next_smooth_time) {
            next_smooth_time = now + 350 + (uint32_t)(random_lcg() % 500);
            uint16_t param = 1000;
            uint16_t min_int = param / 20; // 50
            uint16_t max_int = param / 12; // 83
            if (max_int > min_int) {
                smooth_interval = min_int + (uint16_t)(random_lcg() % (max_int - min_int));
            } else {
                smooth_interval = min_int;
            }
        }

        while (Serial1.available() > 0) {
            int p = Serial1.peek();

            // 1. MAKCU 7-Byte CRC8 Binary Packet (0xA5)
            if (p == 0xA5) {
                if (Serial1.available() >= 7) {
                    uint8_t pkt[7];
                    for (uint8_t i = 0; i < 7; i++) pkt[i] = (uint8_t)Serial1.read();
                    if (computeCrc8(pkt, 6) == pkt[6]) {
                        accum_x += (int16_t)((uint16_t)pkt[1] | ((uint16_t)pkt[2] << 8));
                        accum_y += (int16_t)((uint16_t)pkt[3] | ((uint16_t)pkt[4] << 8));
                        btn_or_mask = pkt[5] & 0x1F;
                        last_move_time = millis();
                        continue;
                    }
                } else {
                    break;
                }
            }

            // 2. MAKCU V2 Binary Packet (0x50)
            if (p == 0x50) {
                if (Serial1.available() >= 5) {
                    Serial1.read(); // 0x50
                    uint8_t mc = (uint8_t)Serial1.read();
                    uint8_t ml = (uint8_t)Serial1.read();
                    Serial1.read(); // len_hi
                    uint8_t p0 = (uint8_t)Serial1.read();
                    if (mc == 0x08) setButtonMaskBit(0x01, p0);
                    else if (mc == 0x11) setButtonMaskBit(0x02, p0);
                    else if (mc == 0x0A) setButtonMaskBit(0x04, p0);
                    else if (mc == 0x12) setButtonMaskBit(0x08, p0);
                    else if (mc == 0x13) setButtonMaskBit(0x10, p0);
                    else if (mc == 0x18) accum_wheel += (int8_t)p0;
                    else if (mc == 0x04 && p0 >= 1 && p0 <= 5) setButtonMaskBit(1 << (p0 - 1), true);
                    else if (mc == 0x0D && ml >= 4 && Serial1.available() >= 3) {
                        uint8_t p1 = (uint8_t)Serial1.read();
                        uint8_t p2 = (uint8_t)Serial1.read();
                        uint8_t p3 = (uint8_t)Serial1.read();
                        accum_x += (int16_t)((uint16_t)p0 | ((uint16_t)p1 << 8));
                        accum_y += (int16_t)((uint16_t)p2 | ((uint16_t)p3 << 8));
                        last_move_time = millis();
                    } else if (mc == 0x0B && ml >= 6 && Serial1.available() >= 5) {
                        btn_or_mask = p0 & 0x1F;
                        int16_t mx = (int16_t)((uint16_t)Serial1.read() | ((uint16_t)Serial1.read() << 8));
                        int16_t my = (int16_t)((uint16_t)Serial1.read() | ((uint16_t)Serial1.read() << 8));
                        accum_wheel += (int8_t)Serial1.read();
                        accum_x += mx; accum_y += my;
                        last_move_time = millis();
                    } else if (mc == 0xBF) {
                        static const char vr[] PROGMEM = "\x50\xBF\x05\x00""4.105";
                        writePgm(vr);
                        continue;
                    } else if (mc == 0xBB) {
                        *(volatile uint16_t *)0x0800 = 0x7777;
                        wdt_enable(WDTO_15MS);
                        for (;;) {}
                    }
                    uint8_t ack[5] = {0x50, mc, 1, 0, 0};
                    Serial1.write(ack, 5);
                    continue;
                } else {
                    break;
                }
            }

            // 3. MAKCU V4 / DE AD Frame
            if (p == 0xDE) {
                if (Serial1.available() >= 8) {
                    Serial1.read(); // 0xDE
                    if (Serial1.read() == 0xAD) {
                        Serial1.read(); Serial1.read(); // len
                        if (Serial1.read() == 0xA5) { // baud change
                            uint32_t b = 0;
                            for (uint8_t i = 0; i < 4; i++) ((uint8_t*)&b)[i] = Serial1.read();
                            if (b >= 9600 && b <= 4000000UL) {
                                Serial1.flush();
                                Serial1.begin(b);
                            }
                        }
                    }
                    continue;
                } else {
                    break;
                }
            }

            // Check for raw Handshake commands via peek
            if (!in_packet) {
                if (p == 0x14) {
                    Serial1.read();
                    handleCmd14();
                    continue;
                }
                if (p == 0x15) {
                    if (Serial1.available() < 33) {
                        break; // Wait for full 33-byte packet
                    }
                    Serial1.read(); // consume 0x15
                    uint8_t payload[32];
                    for (int i = 0; i < 32; i++) {
                        payload[i] = (uint8_t)Serial1.read();
                    }
                    handleCmd15(payload);
                    continue;
                }
            }

            uint8_t b = (uint8_t)Serial1.read();

            // Packet Framing State Machine (0x426e..0x42bc)
            if (b == start_byte) {
                in_packet = true;
                rx_len = 0;
                continue;
            }

            if (b == end_byte) {
                if (in_packet) {
                    in_packet = false;
                    if (rx_len > 0) {
                        uint8_t cmd = rx_buf[0];
                        if (cmd >= 1 && cmd <= 17) {
                            uint8_t payload_len = rx_len - 1;
                            executeCommand(cmd, payload_len, rx_buf);
                        }
                    }
                }
                continue;
            }

            if (in_packet) {
                if (rx_len < RX_BUF_SIZE) {
                    rx_buf[rx_len++] = b;
                }
                continue;
            }

            // Fallback kmbox ASCII Command Parser
            if (b == '\r' || b == '\n') {
                if (ascii_idx > 0) {
                    ascii_buf[ascii_idx] = '\0';
                    parseKmboxAscii(ascii_buf);
                    ascii_idx = 0;
                }
            } else if (ascii_idx < (ASCII_BUF_SIZE - 1)) {
                if (b >= 0x20 && b <= 0x7E) {
                    ascii_buf[ascii_idx++] = (char)b;
                }
            } else {
                ascii_idx = 0;
            }
        }
    }

private:
    // ------------------------------------------------------------------------
    // Handshake 0x14 (0x4e3e): Challenge Request -> Response Packet
    // ------------------------------------------------------------------------
    void __attribute__((noinline)) handleCmd14() {
        uint64_t h = compute_cmd14_hash(nonce, 16);
        uint8_t pkt[26];
        pkt[0] = 0xAC;
        memcpy(pkt + 1, nonce, 16);
        memcpy(pkt + 17, &h, 8);
        pkt[25] = 0xBD;
        Serial1.write(pkt, 26);
    }

    // ------------------------------------------------------------------------
    // Handshake 0x15 (0x4edc..0x56d0): Challenge Response Verification
    // ------------------------------------------------------------------------
    void __attribute__((noinline)) handleCmd15(uint8_t *p) {
        // Decrypt both 16-byte blocks in place
        block_decrypt_1988(&p[0], &crypto_state);
        block_decrypt_1988(&p[16], &crypto_state);

        // 1. Verify standard FNV-1a 64-bit hash over P[0..23]
        uint64_t h = 0xCBF29CE484222325ULL;
        for (uint8_t i = 0; i < 24; i++) {
            h ^= p[i];
            h *= 1099511628211ULL;
        }

        uint64_t expected_hash = 0;
        for (uint8_t i = 0; i < 8; i++) {
            expected_hash = (expected_hash << 8) | (uint64_t)p[24 + i];
        }

        if (h != expected_hash) {
            auth_counter++;
            return;
        }

        // 2. Derive 16-byte key from nonce
        uint8_t stack_16[16];
        handshake_derive_nonce_bytes(nonce, stack_16);

        if (memcmp(p, stack_16, 16) != 0) {
            auth_counter++;
            return;
        }

        // 3. Extract dynamic control bytes (rol 3 bits: (x << 3) | (x >> 5))
        uint8_t rol_s13 = (uint8_t)((stack_16[13] << 3) | (stack_16[13] >> 5));
        uint8_t rol_s0  = (uint8_t)((stack_16[0]  << 3) | (stack_16[0]  >> 5));
        uint8_t rol_s5  = (uint8_t)((stack_16[5]  << 3) | (stack_16[5]  >> 5));

        // Subsequent one-time transformations (disassembly 0x41cc, 0x4210, 0x4d42)
        base_button_mask = xen_deobfuscate((uint8_t)((p[16] - rol_s13) ^ 0x5C));
        start_byte       = xen_deobfuscate((uint8_t)((p[18] - rol_s0)  ^ 0x5C));
        end_byte         = xen_deobfuscate((uint8_t)((p[19] - rol_s5)  ^ 0x5C));

        authenticated = 1;
        auth_counter = 0;
        Serial1.setTimeout(1);
    }

    // ------------------------------------------------------------------------
    // Jump Table Dispatcher for Commands 1..17 (0x42c8)
    // ------------------------------------------------------------------------
    void __attribute__((noinline)) executeCommand(uint8_t cmd, uint8_t payload_len, const uint8_t *buf) {
        // buf[0] == cmd
        // buf[1..payload_len] == payload
        switch (cmd) {
            case 1: { // 0x42EA: Button Press / Release
                if (payload_len >= 2) {
                    uint8_t mask = buf[1];
                    uint8_t action = buf[2];
                    if (action == 1) {
                        btn_or_mask |= mask;
                    } else if (action == 0) {
                        btn_or_mask &= ~mask;
                    }
                }
                break;
            }

            case 2: { // 0x4308: NOP
                break;
            }

            case 3:   // 0x4314: Relative Mouse Move (8-bit)
            case 7: { // 0x43B6: Combined 8-bit Mouse Report
                if (cmd == 7) {
                    if (payload_len < 4) break;
                    btn_or_mask = buf[1];
                    accum_wheel += (int8_t)buf[4];
                    buf++;
                } else if (payload_len < 2) break;
                accum_x += (int8_t)buf[1];
                accum_y += (int8_t)buf[2];
                last_move_time = millis();
                break;
            }

            case 4:   // 0x4352: Relative Mouse Move (16-bit little-endian)
            case 8: { // 0x4406: Combined 16-bit Mouse Report
                if (cmd == 8) {
                    if (payload_len < 6) break;
                    btn_or_mask = buf[1];
                    accum_wheel += (int8_t)buf[6];
                    buf++;
                } else if (payload_len < 4) break;
                accum_x += (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
                accum_y += (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
                last_move_time = millis();
                break;
            }

            case 5: { // 0x4390: Mouse Scroll Wheel
                if (payload_len >= 1) {
                    accum_wheel += (int8_t)buf[1];
                }
                break;
            }

            case 6: { // 0x43A6: Set Button Clear/Block Mask
                if (payload_len >= 1) {
                    btn_block_mask = buf[1];
                }
                break;
            }

            case 9: { // 0x444E: Set Button Monitoring Flag
                if (payload_len >= 1) {
                    btn_report_enabled = (buf[1] != 0) ? 1 : 0;
                }
                break;
            }

            case 10: { // 0x4466: Ping
                if (payload_len >= 1 && buf[1] == 0xFF) {
                    uint8_t resp = 0x69; // 'i'
                    Serial1.write(&resp, 1);
                }
                break;
            }

            case 11: { // 0x4484: Read Mode from EEPROM (0x03FF)
                uint8_t mode = eeprom_read_byte((const uint8_t *)0x03FF);
                Serial1.write(&mode, 1);
                break;
            }

            case 12: { // 0x448E: Set Mode in EEPROM (0x03FF)
                if (payload_len >= 1) {
                    uint8_t mode = buf[1];
                    if (mode == 0x01 || mode == 0xFF) {
                        eeprom_update_byte((uint8_t *)0x03FF, mode);
                    }
                }
                break;
            }

            case 13: { // 0x44C6: Query Device Descriptor
                static const char dev_desc[] PROGMEM = "\x03XEN-LEONARDO-HID";
                for (uint8_t i = 0; i < sizeof(dev_desc) - 1; i++) {
                    Serial1.write((uint8_t)pgm_read_byte(&dev_desc[i]));
                }
                break;
            }

            case 14: { // 0x458C: Authenticated Profile Upload (7 slots x 131 bytes)
                if (payload_len >= 2) {
                    uint8_t cfg_len = buf[1];
                    for (uint8_t slot = 0; slot < 7; slot++) {
                        uint16_t addr = (uint16_t)slot * 131;
                        if (eeprom_read_byte((const uint8_t *)addr) != 0xAB) {
                            eeprom_update_byte((uint8_t *)addr, 0xAB);
                            eeprom_update_byte((uint8_t *)(addr + 1), cfg_len);
                            for (uint8_t k = 0; k < cfg_len && k < 129; k++) {
                                eeprom_update_byte((uint8_t *)(addr + 2 + k), buf[2 + k]);
                            }
                            break;
                        }
                    }
                }
                break;
            }

            case 15: { // 0x48E0: Delete Profile from EEPROM
                for (uint8_t slot = 0; slot < 7; slot++) {
                    uint16_t addr = (uint16_t)slot * 131;
                    if (eeprom_read_byte((const uint8_t *)addr) == 0xAB) {
                        for (uint8_t k = 0; k < 131; k++) {
                            eeprom_update_byte((uint8_t *)(addr + k), 0);
                        }
                        break;
                    }
                }
                break;
            }

            case 16: { // 0x4998: Query Status Report
                sendStatusReport();
                break;
            }

            case 17: { // 0x496E: Set Smoothing Enable in EEPROM (0x03FE)
                if (payload_len >= 1) {
                    uint8_t val = buf[1];
                    if (val == 0x01 || val == 0xFF) {
                        eeprom_update_byte((uint8_t *)0x03FE, val);
                        smoothing_enabled = (val == 0x01) ? 1 : 0;
                        sendStatusReport();
                    }
                }
                break;
            }

            default:
                break;
        }
    }

    void sendStatusReport() {
        uint8_t mode = (eeprom_read_byte((const uint8_t *)0x03FF) == 1) ? 1 : 0;
        uint8_t smoothing = (eeprom_read_byte((const uint8_t *)0x03FE) == 1) ? 1 : 0;
        smoothing_enabled = smoothing;

        uint8_t rep[4] = {0x02, 0x00, mode, smoothing};
        Serial1.write(rep, 4);
    }

    // ------------------------------------------------------------------------
    // Smoothing Subsystem (Disassembly 0x10b2..0x12ac & 0x49c0..0x4b24)
    // ------------------------------------------------------------------------
    static void __attribute__((noinline)) smooth_axis(int16_t *local_val, int16_t *accum_val,
                            int8_t *state, uint32_t *last_step_time, uint16_t smooth_int,
                            uint32_t now) {
        if (*local_val == 0) return;
        int8_t step_sign = (*local_val > 0) ? 1 : -1;

        if (*state != 0 && *state != step_sign) {
            if ((uint32_t)(now - *last_step_time) < smooth_int) {
                *accum_val = 0;
                *local_val = 0;
                return;
            } else {
                *last_step_time = now;
                *state = step_sign;
                *local_val = *accum_val;
            }
        } else {
            *state = step_sign;
            *local_val = *accum_val;
        }
    }

    void applySmoothing(uint32_t now, int16_t &dx, int16_t &dy) {
        if ((uint32_t)(now - last_move_time) > smooth_interval) {
            return;
        }
        smooth_axis(&dx, &accum_x, &state_x, &last_step_time_x, smooth_interval, now);
        smooth_axis(&dy, &accum_y, &state_y, &last_step_time_y, smooth_interval, now);
    }

    // ------------------------------------------------------------------------
    // Standard kmbox ASCII Command Parser
    // ------------------------------------------------------------------------
    static void skipSpaces(const char *&p) {
        while (*p == ' ' || *p == '\t') p++;
    }

    static int32_t parseI32(const char *&p) {
        skipSpaces(p);
        bool neg = (*p == '-');
        if (neg || *p == '+') p++;
        int32_t v = 0;
        while (*p >= '0' && *p <= '9') {
            v = v * 10 + (*p++ - '0');
        }
        return neg ? -v : v;
    }

    static inline int16_t parseI16(const char *&p) {
        return (int16_t)parseI32(p);
    }

    static uint8_t parseU8(const char *&p) {
        skipSpaces(p);
        uint8_t base = 10;
        if (p[0] == '0' && (p[1] | 0x20) == 'x') {
            base = 16;
            p += 2;
        }
        uint8_t v = 0;
        while (true) {
            char c = *p;
            if (c >= '0' && c <= '9') { v = v * base + (c - '0'); p++; }
            else if (base == 16 && ((c | 0x20) >= 'a' && (c | 0x20) <= 'f')) { v = v * 16 + ((c | 0x20) - 'a' + 10); p++; }
            else break;
        }
        return v;
    }

    static void writePgm(const char *pgm_s) {
        char c;
        while ((c = (char)pgm_read_byte(pgm_s++))) {
            Serial1.write((uint8_t)c);
        }
    }

    static void replyKmOk() {
        static const char ok_str[] PROGMEM = "km.ok\r\n";
        writePgm(ok_str);
    }

    static const char* findChar(const char *s, char c) {
        while (*s && *s != c) s++;
        return (*s == c) ? s : NULL;
    }

    void __attribute__((noinline)) parseKmboxAscii(const char *cmd) {
        const char *cptr = cmd;
        skipSpaces(cptr);
        if (cptr[0] == 'p' && cptr[1] == 'i') {
            replyKmOk();
            return;
        }
        if (cptr[0] == '.') {
            cptr += 1;
        } else if (cptr[0] == 'k' && cptr[1] == 'm' && cptr[2] == '.') {
            cptr += 3;
        } else {
            return;
        }

        char c = cptr[0] | 0x20;
        char c4 = cptr[1] | 0x20;
        const char *p = findChar(cptr, '(');
        if (p) p++;

        uint8_t btn_bit = 0;
        if (c == 'l' && c4 == 'e') btn_bit = 0x01;
        else if (c == 'r') btn_bit = 0x02;
        else if (c == 'm' && c4 == 'i') btn_bit = 0x04;
        else if (c == 's') btn_bit = (cptr[4] == '2' || (p && *(p-2) == '2')) ? 0x10 : 0x08;

        if (btn_bit) {
            skipSpaces(p);
            if (*p == ')' || *p == '\0') {
                uint8_t st = (btn_or_mask & btn_bit) ? 2 : ((last_raw_phys_buttons & btn_bit) ? 1 : 0);
                static const char p_km[] PROGMEM = "km.";
                static const char p_l[] PROGMEM = "left(";
                static const char p_r[] PROGMEM = "right(";
                static const char p_m[] PROGMEM = "middle(";
                static const char p_s[] PROGMEM = "side1(";
                writePgm(p_km);
                writePgm((btn_bit==1)?p_l:(btn_bit==2)?p_r:(btn_bit==4)?p_m:p_s);
                Serial1.write((uint8_t)('0' + st));
                Serial1.write((const uint8_t*)")\r\n", 3);
                return;
            }
            setButtonMaskBit(btn_bit, parseI16(p) != 0);
        } else if (c == 'm') {
            if (c4 == 'o') { // km.move(x,y) or km.mo(...)
                if (cptr[2] == '(') { // km.mo(buttons,x,y,wheel,pan,tilt)
                    btn_or_mask = parseU8(p) & 0x1F;
                    if (*p == ',') p++;
                    accum_x += parseI16(p);
                    if (*p == ',') p++;
                    accum_y += parseI16(p);
                    if (*p == ',') p++;
                    accum_wheel += (int8_t)parseI16(p);
                    last_move_time = millis();
                } else {
                    accum_x += parseI16(p);
                    if (*p == ',') p++;
                    accum_y += parseI16(p);
                    last_move_time = millis();
                }
            } else if (c4 == 'a') { // km.mask(or,block)
                btn_or_mask = parseU8(p) & 0x1F;
                if (*p == ',') p++;
                btn_block_mask = parseU8(p) & 0x1F;
            }
        } else if (c == 'l' && c4 == 'i') { // km.listen(v)
            btn_report_enabled = (parseI16(p) != 0) ? 1 : 0;
        } else if (c == 'w') { // km.wheel(w)
            accum_wheel += (int8_t)parseI16(p);
        } else if (c == 'c') { // km.click(btn)
            uint8_t b = parseU8(p);
            if (b >= 1 && b <= 5) setButtonMaskBit(1 << (b - 1), true);
            else setButtonMaskBit(0x01, true);
        } else if (c == 'u') { // km.unmask()
            btn_or_mask = 0;
            btn_block_mask = 0;
        } else if (c == 'v') { // km.version()
            static const char p_ver[] PROGMEM = "km.version(4.105)\r\n";
            writePgm(p_ver);
            return;
        } else if (c == 'i') { // km.init()
            btn_or_mask = 0;
            btn_block_mask = 0;
            accum_x = 0;
            accum_y = 0;
            accum_wheel = 0;
        } else if (c == 'b') { // km.baud(rate) or km.boot()
            if (c4 == 'o') {
                *(volatile uint16_t *)0x0800 = 0x7777;
                wdt_enable(WDTO_15MS);
                for (;;) {}
            }
            int32_t baud = parseI32(p);
            if (baud > 0) {
                replyKmOk();
                Serial1.flush();
                Serial1.begin(baud);
                return;
            }
        } else if (c != 'p') {
            return;
        }
        replyKmOk();
    }

    void setButtonMaskBit(uint8_t bit, bool press) {
        if (press) {
            btn_or_mask |= bit;
            btn_block_mask &= ~bit;
        } else {
            btn_or_mask &= ~bit;
        }
    }
};

static XenKmboxEngine Xen;

// ============================================================================
// 6. Universal Multi-Interface USB Host Mouse Driver
// ============================================================================
class UniversalMouse : public HIDComposite {
private:
    MouseReportLayout layouts[maxHidInterfaces];
    bool has_any_valid_layout = false;
    uint32_t last_poll_us = 0;
    uint8_t consecutive_errors = 0;
    uint32_t last_activity_ms = 0;

    void clearEndpointHalt(uint8_t epAddr) {
        pUsb->ctrlReq(bAddress, 0,
                      USB_SETUP_HOST_TO_DEVICE | USB_SETUP_TYPE_STANDARD | USB_SETUP_RECIPIENT_ENDPOINT,
                      USB_REQUEST_CLEAR_FEATURE, USB_FEATURE_ENDPOINT_HALT, 0,
                      epAddr | 0x80, 0, 0, NULL, NULL);
    }

    void dispatchReport(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel) {
        uint8_t out_btn = 0;
        int16_t out_dx = 0, out_dy = 0;
        int8_t out_w = 0;
        Xen.processPhysicalReport(buttons, dx, dy, wheel, out_btn, out_dx, out_dy, out_w);
        PcMouse.feedInput(out_btn, out_dx, out_dy, out_w);
        PcMouse.flushOnce();
        last_activity_ms = millis();
    }

    void parseWithFallback(uint8_t len, const uint8_t *buf) {
        if (len < 3) return;

        uint8_t buttons = 0;
        int16_t dx = 0;
        int16_t dy = 0;
        int8_t wheel = 0;

        if (len == 3) {
            buttons = buf[0] & 0x1F;
            dx = (int8_t)buf[1];
            dy = (int8_t)buf[2];
        } else if (len == 4) {
            buttons = buf[0] & 0x1F;
            dx = (int8_t)buf[1];
            dy = (int8_t)buf[2];
            wheel = (int8_t)buf[3];
        } else if (len == 5) {
            if (buf[0] >= 1 && buf[0] <= 4 && (buf[1] & 0xE0) == 0) {
                buttons = buf[1] & 0x1F;
                dx = (int8_t)buf[2];
                dy = (int8_t)buf[3];
                wheel = (int8_t)buf[4];
            } else {
                buttons = buf[0] & 0x1F;
                dx = extractSignedBits(buf, len, 8, 12);
                dy = extractSignedBits(buf, len, 20, 12);
                wheel = (int8_t)buf[4];
            }
        } else if (len >= 6) {
            buttons = buf[0] & 0x1F;
            dx = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
            dy = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
            wheel = (int8_t)buf[5];
        }

        dispatchReport(buttons, dx, dy, wheel);
    }

    void processInterfaceReport(uint8_t iface_idx, uint8_t len, const uint8_t *buf) {
        if (len == 0) return;

        const MouseReportLayout &lyt = layouts[iface_idx];
        if (!lyt.valid) {
            if (lyt.is_fallback) {
                parseWithFallback(len, buf);
            }
            return;
        }

        if (lyt.has_report_id) {
            if (buf[0] != lyt.report_id) {
                return;
            }
        }

        uint16_t min_bits = lyt.y_offset + lyt.y_size;
        if (lyt.x_offset + lyt.x_size > min_bits) {
            min_bits = lyt.x_offset + lyt.x_size;
        }
        if (((uint16_t)len << 3) < min_bits) {
            return;
        }

        uint8_t buttons = 0;
        if (lyt.btn_size > 0) {
            buttons = extractUnsignedBits(buf, len, lyt.btn_offset, lyt.btn_size);
        } else {
            uint8_t btn_byte = lyt.has_report_id ? (len > 1 ? buf[1] : 0) : buf[0];
            buttons = btn_byte & 0x1F;
        }

        int16_t dx = extractSignedBits(buf, len, lyt.x_offset, lyt.x_size);
        int16_t dy = extractSignedBits(buf, len, lyt.y_offset, lyt.y_size);
        int8_t wheel = 0;
        if (lyt.wheel_size > 0 && ((uint16_t)len << 3) >= (lyt.wheel_offset + lyt.wheel_size)) {
            int16_t w = extractSignedBits(buf, len, lyt.wheel_offset, lyt.wheel_size);
            if (w > 127) w = 127;
            if (w < -127) w = -127;
            wheel = (int8_t)w;
        }

        dispatchReport(buttons, dx, dy, wheel);
    }

protected:
    bool SelectInterface(uint8_t iface __attribute__((unused)), uint8_t proto __attribute__((unused))) override {
        return true;
    }

    void resetEp0() {
        wdt_reset();
        epInfo[0].bmSndToggle = 0;
        epInfo[0].bmRcvToggle = 0;
    }

    uint8_t OnInitSuccessful() override {
        has_any_valid_layout = false;
        consecutive_errors = 0;

        for (uint8_t i = 0; i < maxHidInterfaces; i++) {
            layouts[i].clear();
        }

        for (uint8_t i = 0; i < bNumIface; i++) {
            wdt_reset();
            if (hidInterfaces[i].epIndex[epInterruptInIndex] == 0) {
                continue;
            }

            uint8_t iface_num = hidInterfaces[i].bmInterface;
            resetEp0();
            SetProtocol(iface_num, HID_RPT_PROTOCOL);

            uint8_t buf[64];
            for (uint8_t attempt = 0; attempt < 3; attempt++) {
                resetEp0();
                HidReportDescParser prs(layouts[i]);
                uint8_t rcode = pUsb->ctrlReq(bAddress, 0x00, bmREQ_HID_REPORT, USB_REQUEST_GET_DESCRIPTOR,
                                              0x00, HID_DESCRIPTOR_REPORT, iface_num, (attempt == 0) ? 383 : 128, sizeof(buf), buf, &prs);
                if (rcode == 0 || layouts[i].valid) break;
                delay(20);
            }

            if (layouts[i].valid) {
                has_any_valid_layout = true;
            } else if (hidInterfaces[i].bmProtocol == USB_HID_PROTOCOL_MOUSE) {
                layouts[i].is_fallback = true;
            }
        }

        if (!has_any_valid_layout) {
            for (uint8_t i = 0; i < bNumIface; i++) {
                if (hidInterfaces[i].epIndex[epInterruptInIndex] != 0) {
                    layouts[i].is_fallback = true;
                }
            }
        }

        pollInterval = 1;
        qNextPollTime = millis();
        last_poll_us = micros();
        return 0;
    }

public:
    UniversalMouse(USB *p) : HIDComposite(p) {
        for (uint8_t i = 0; i < maxHidInterfaces; i++) {
            layouts[i].clear();
        }
    }

    uint32_t getLastActivityMs() const {
        return last_activity_ms;
    }

    bool probeBus() {
        pUsb->regWr(rHCTL, bmSAMPLEBUS);
        uint8_t timeout = 250;
        while (!(pUsb->regRd(rHCTL) & bmSAMPLEBUS) && --timeout) {
            delayMicroseconds(1);
        }
        pUsb->busprobe();
        uint8_t vbus = pUsb->getVbusState();
        return (vbus == FSHOST || vbus == LSHOST);
    }

    void probeAndRecover() {
        Release();
        AddressPool &addrPool = pUsb->GetAddressPool();
        for (uint8_t a = 1; a < USB_NUMDEVICES; a++) {
            addrPool.FreeAddress(a);
        }
        pUsb->setUsbTaskState(USB_DETACHED_SUBSTATE_INITIALIZE);
        probeBus();
    }

    uint8_t getDescrRetry(uint8_t addr, uint8_t nbytes, uint8_t *buf, uint8_t d) {
        uint8_t r = 0;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            resetEp0();
            r = pUsb->getDevDescr(addr, 0, nbytes, buf);
            if (!r) break;
            delay(d);
        }
        return r;
    }

    uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed) override {
        const uint8_t constBufSize = sizeof(USB_DEVICE_DESCRIPTOR);
        uint8_t buf[constBufSize];
        USB_DEVICE_DESCRIPTOR *udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR *>(buf);
        uint8_t rcode = 0;
        UsbDevice *p = NULL;
        EpInfo *oldep_ptr = NULL;
        uint8_t len = 0;
        uint8_t num_of_conf = 0;

        AddressPool &addrPool = pUsb->GetAddressPool();

        if (bAddress) {
            return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
        }

        Initialize();

        // 100ms post-bus-reset recovery delay (10 x 10ms with watchdog reset)
        // Accommodates slow POR stabilization on low-speed mice and ARM boot on gaming mice
        for (uint8_t w = 0; w < 10; w++) {
            wdt_reset();
            delay(10);
        }

        // Verify lowspeed setting from actual vbusState / MAX3421E mode register
        uint8_t vbus = pUsb->getVbusState();
        if (vbus == LSHOST || (pUsb->regRd(rMODE) & bmLOWSPEED) != 0) {
            lowspeed = true;
        } else if (vbus == FSHOST) {
            lowspeed = false;
        }

        p = addrPool.GetUsbDevicePtr(0);
        if (!p) return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        if (!p->epinfo) return USB_ERROR_EPINFO_IS_NULL;

        oldep_ptr = p->epinfo;
        p->epinfo = epInfo;
        p->lowspeed = lowspeed;

        // Stage 1: Initial 8-byte Device Descriptor read (3 retries)
        rcode = getDescrRetry(0, 8, buf, 25);

        if (!rcode) {
            len = (buf[0] > constBufSize) ? constBufSize : buf[0];
        }
        p->epinfo = oldep_ptr;

        if (rcode) {
            Release();
            return rcode;
        }

        bAddress = addrPool.AllocAddress(parent, false, port);
        if (!bAddress) {
            return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;
        }

        epInfo[0].maxPktSize = udd->bMaxPacketSize0;

        // Stage 2: SetAddress with 3 retries
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            resetEp0();
            rcode = pUsb->setAddr(0, 0, bAddress);
            if (!rcode) break;
            delay(20);
        }

        if (rcode) {
            p->lowspeed = false;
            addrPool.FreeAddress(bAddress);
            bAddress = 0;
            return rcode;
        }

        // 15ms SetAddress recovery delay
        delay(15);
        wdt_reset();

        p->lowspeed = false;
        p = addrPool.GetUsbDevicePtr(bAddress);
        if (!p) {
            Release();
            return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
        }
        p->lowspeed = lowspeed;

        // Ensure epInfo is registered at the newly allocated address before descriptor query
        rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
        if (rcode) {
            Release();
            return rcode;
        }

        // Stage 3: Full 18-byte Device Descriptor read with 3 retries
        if (len) {
            rcode = getDescrRetry(bAddress, len, buf, 20);
        }

        if (rcode) {
            Release();
            return rcode;
        }

        VID = udd->idVendor;
        PID = udd->idProduct;
        num_of_conf = udd->bNumConfigurations;

        // Stage 4: Parse Configuration Descriptor with 3 retries per configuration
        for (uint8_t i = 0; i < num_of_conf; i++) {
            for (uint8_t attempt = 0; attempt < 3; attempt++) {
                resetEp0();
                ConfigDescParser<USB_CLASS_HID, 0, 0, CP_MASK_COMPARE_CLASS> confDescrParser(this);
                rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
                if (!rcode) break;
                delay(25);
            }
            if (rcode) {
                Release();
                return rcode;
            }
            if (bNumEP > 1) break;
        }

        if (bNumEP < 2) {
            Release();
            return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
        }

        rcode = pUsb->setEpInfoEntry(bAddress, bNumEP, epInfo);
        if (rcode) {
            Release();
            return rcode;
        }

        // Stage 5: SetConfiguration with 3 retries
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            resetEp0();
            rcode = pUsb->setConf(bAddress, 0, bConfNum);
            if (!rcode) break;
            delay(20);
        }

        if (rcode) {
            Release();
            return rcode;
        }

        // 15ms SetConfiguration recovery delay
        delay(15);
        wdt_reset();

        for (uint8_t i = 0; i < bNumIface; i++) {
            if (hidInterfaces[i].epIndex[epInterruptInIndex] == 0) continue;
            SetIdle(hidInterfaces[i].bmInterface, 0, 0);
        }

        OnInitSuccessful();
        bPollEnable = true;
        return 0;
    }

    uint8_t Release() override {
        PcMouse.resetState();
        has_any_valid_layout = false;
        consecutive_errors = 0;
        for (uint8_t i = 0; i < maxHidInterfaces; i++) {
            layouts[i].clear();
        }
        return HIDComposite::Release();
    }

    uint8_t Poll() override {
        if (!bPollEnable) return 0;

        uint32_t now_us = micros();
        if ((uint32_t)(now_us - last_poll_us) < 950UL) {
            return 0;
        }
        last_poll_us = now_us;

        uint8_t buf[constBuffLen];

        for (uint8_t i = 0; i < bNumIface; i++) {
            uint8_t index = hidInterfaces[i].epIndex[epInterruptInIndex];
            if (index == 0) continue;

            if (has_any_valid_layout && !layouts[i].valid) {
                continue;
            }

            uint16_t read = (uint16_t)epInfo[index].maxPktSize;
            if (read > constBuffLen) read = constBuffLen;

            uint8_t rcode = pUsb->inTransfer(bAddress, epInfo[index].epAddr, &read, buf);

            if (rcode == 0) {
                consecutive_errors = 0;
                if (read > 0) {
                    if (read > constBuffLen) read = constBuffLen;
                    processInterfaceReport(i, (uint8_t)read, buf);
                }
            } else if (rcode == hrNAK) {
                consecutive_errors = 0;
            } else if (rcode == hrSTALL) {
                clearEndpointHalt(epInfo[index].epAddr);
                epInfo[index].bmRcvToggle = 0;
                consecutive_errors++;
            } else {
                consecutive_errors++;
            }
        }

        if (consecutive_errors > 40) {
            probeAndRecover();
        }

        return 0;
    }
};

// ============================================================================
// 7. Main Application & State Machine Watchdog
// ============================================================================
USB Usb;
USBHub Hub(&Usb);
UniversalMouse HostMouse(&Usb);

static uint8_t prev_usb_state = 0xFF;
static uint32_t state_enter_ms = 0;
static bool shield_ok = false;

void setup() {
    // Hardware Watchdog (2-second timeout)
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    // Initialize USB Host Shield
    if (Usb.Init() == -1) {
        shield_ok = false;
    } else {
        shield_ok = true;
        digitalWrite(13, HIGH);
    }

    // Initialize XEN Co-Processor & kmbox on Serial1 (115200 baud)
    Xen.begin(115200);

    wdt_enable(WDTO_2S);
    state_enter_ms = millis();
}

void loop() {
    wdt_reset();

    // Process incoming serial commands on Serial1
    Xen.task();

    // Standalone synthetic report evaluation when physical mouse is idle
    uint8_t st_btn = 0;
    int16_t st_dx = 0, st_dy = 0;
    int8_t st_w = 0;
    if (Xen.evaluateStandaloneReport(st_btn, st_dx, st_dy, st_w)) {
        PcMouse.feedInput(st_btn, st_dx, st_dy, st_w);
        PcMouse.flushOnce();
    }

    if (!shield_ok) {
        static uint32_t last_retry = 0;
        if (millis() - last_retry >= 500) {
            last_retry = millis();
            digitalWrite(13, !digitalRead(13));
            if (Usb.Init() != -1) {
                shield_ok = true;
            }
        }
        PcMouse.flushOnce();
        return;
    }

    Usb.Task();

    // Flush any remaining movement deltas to PC
    PcMouse.flushOnce();

    uint8_t state = Usb.getUsbTaskState();
    uint32_t now = millis();

    if (state != prev_usb_state) {
        prev_usb_state = state;
        state_enter_ms = now;
    }

    // Active bus probe in DETACHED wait / illegal states:
    // If waiting for device >= 250ms, actively sample the MAX3421E USB bus.
    // If J-status or K-status is active, mouse is physically connected.
    // busprobe() updates vbusState; Usb.Task() transitions to USB_ATTACHED_SUBSTATE_SETTLE naturally.
    if (state == USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE || state == USB_DETACHED_SUBSTATE_ILLEGAL) {
        if (now - state_enter_ms >= 250) {
            state_enter_ms = now;
            HostMouse.probeBus();
        }
    }

    // State machine recovery watchdog
    if (state == USB_STATE_ERROR || state == USB_DETACHED_SUBSTATE_ILLEGAL) {
        if (now - state_enter_ms >= 120) {
            HostMouse.probeAndRecover();
            state_enter_ms = now;
        }
    } else if (state != USB_STATE_RUNNING && state != USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE) {
        if (now - state_enter_ms >= 1500) {
            HostMouse.probeAndRecover();
            state_enter_ms = now;
        }
    }

    // LED status indicator
    if (state == USB_STATE_RUNNING) {
        if (now - HostMouse.getLastActivityMs() < 25) {
            digitalWrite(13, (now & 0x10) ? HIGH : LOW);
        } else {
            digitalWrite(13, HIGH);
        }
    } else {
        digitalWrite(13, ((now >> 9) & 1) ? HIGH : LOW);
    }
}

