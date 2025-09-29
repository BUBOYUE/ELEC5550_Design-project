#include <Arduino.h>
#include <string.h>
#include "uart.h"

// ===================== 基础：串口与 CRC =====================
void uart2_init() {
  // ESP32 的 Serial2: (baud, config, RX, TX)
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(50);
}

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  const uint16_t poly = 0x1021;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ poly;
      else              crc <<= 1;
    }
  }
  return crc;
}

// 覆盖 TYPE|LEN|PAYLOAD 的 CRC，便于调用
static uint16_t crc_type_len_payload(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t tmp[2 + 255]; // LEN 最大255
  tmp[0] = type;
  tmp[1] = len;
  if (len && payload) memcpy(&tmp[2], payload, len);
  return crc16_ccitt_false(tmp, 2 + len);
}

// ===================== 裸帧层（与你原来完全一致） =====================
// 我把你原来的 send_frame/read_frame 改名为 *_raw，逻辑不变

// 帧格式: [STX][TYPE][LEN][PAYLOAD..][CRC_H][CRC_L]
static bool send_frame_raw(uint8_t type, const uint8_t* payload, uint8_t len) {
  const size_t total = 1 + 1 + 1 + len + 2;
  uint8_t buf[1 + 1 + 1 + 255 + 2]; // STX + TYPE + LEN + payload + CRC
  buf[0] = STX;
  buf[1] = type;
  buf[2] = len;
  if (len && payload) memcpy(&buf[3], payload, len);

  uint16_t crc = crc_type_len_payload(type, len, payload);
  buf[3 + len + 0] = (crc >> 8) & 0xFF;
  buf[3 + len + 1] = (crc >> 0) & 0xFF;

  size_t written = Serial2.write(buf, total);
  return (written == total);
}

static bool read_frame_raw(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  enum State { WAIT_STX, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC_H, WAIT_CRC_L };
  static State state = WAIT_STX;
  static uint8_t s_type = 0;
  static uint8_t s_len = 0;
  static uint8_t s_idx = 0;
  static uint8_t s_payload[255];
  static uint8_t s_crc_h = 0;

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    switch (state) {
      case WAIT_STX:
        if (c == STX) state = WAIT_TYPE;
        break;

      case WAIT_TYPE:
        s_type = c;
        state = WAIT_LEN;
        break;

      case WAIT_LEN:
        s_len = c;
        if (s_len == 0) {
          state = WAIT_CRC_H;
        } else if (s_len > sizeof(s_payload)) {
          state = WAIT_STX; // 非法长度，丢弃
        } else {
          s_idx = 0;
          state = WAIT_PAYLOAD;
        }
        break;

      case WAIT_PAYLOAD:
        s_payload[s_idx++] = c;
        if (s_idx >= s_len) state = WAIT_CRC_H;
        break;

      case WAIT_CRC_H:
        s_crc_h = c;
        state = WAIT_CRC_L;
        break;

      case WAIT_CRC_L: {
        uint16_t crc_rx = ((uint16_t)s_crc_h << 8) | c;
        uint16_t crc_calc = crc_type_len_payload(s_type, s_len, s_len ? s_payload : nullptr);

        state = WAIT_STX; // 重置状态机

        if (crc_rx == crc_calc) {
          type = s_type;
          len  = s_len;
          if (payload && max_len >= s_len && s_len) memcpy(payload, s_payload, s_len);
          return true;
        }
        // CRC 错误丢弃
        break;
      }
    }
  }
  return false;
}

// ===================== 可靠层（停等式 ARQ） =====================
// 关键点：把 1bit 序号藏在 TYPE 的最高位；base_type = TYPE & 0x7F

// 发送端序号（每成功一次翻转 0<->1）；接收端期望序号
static uint8_t s_tx_seq = 0;
static uint8_t s_expected_rx_seq = 0;

// 将 base_type 与序号合成为线上发送的 wire_type（最高位是 seq）
static inline uint8_t wire_from(uint8_t base_type, uint8_t seq) {
  return (uint8_t)((base_type & 0x7F) | ((seq & 0x01) << 7));
}
static inline uint8_t base_from(uint8_t wire_type) { return (uint8_t)(wire_type & 0x7F); }
static inline uint8_t seq_from (uint8_t wire_type) { return (uint8_t)((wire_type >> 7) & 0x01); }

// 发送 ACK/NAK（payload[0] = seq）
static inline void send_ack(uint8_t seq) {
  uint8_t p[1] = { (uint8_t)(seq & 0x01) };
  (void)send_frame_raw(MSG_ACK, p, 1);
}
static inline void send_nak(uint8_t seq) {
  uint8_t p[1] = { (uint8_t)(seq & 0x01) };
  (void)send_frame_raw(MSG_NAK, p, 1);
}

// 等待 ACK(seq)；期间如果读到别的帧，丢弃/忽略即可
static bool wait_ack(uint8_t want_seq, uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeout_ms) {
    uint8_t t, l, buf[8];
    if (read_frame_raw(t, buf, l, sizeof(buf))) {
      if (t == MSG_ACK && l == 1 && (buf[0] & 0x01) == want_seq) {
        return true;
      }
      // 其它帧：这里按需处理或忽略
    }
    delay(0); // 让出调度
  }
  return false; // 超时
}

// 可靠发送：发 wire_type，等 ACK(seq)，超时/NAK 重传
static bool send_frame_reliable(uint8_t base_type, const uint8_t* payload, uint8_t len,
                                uint32_t timeout_ms = 20, uint8_t max_retries = 5) {
  uint8_t seq = s_tx_seq & 0x01;
  uint8_t wire_type = wire_from(base_type, seq);

  for (uint8_t attempt = 0; attempt <= max_retries; ++attempt) {
    if (!send_frame_raw(wire_type, payload, len)) {
      delay(2);       // 串口拥塞，小退避
      continue;
    }
    if (wait_ack(seq, timeout_ms)) {
      s_tx_seq ^= 1; // 成功收到 ACK，再翻转序号
      return true;
    }
    // 超时未等到 ACK -> 重传；seq 不变
  }
  return false;
}

// 可靠接收：去重 + 回 ACK
static bool read_frame_reliable(uint8_t& base_type, uint8_t* payload, uint8_t& len, size_t max_len) {
  uint8_t wire_type, L;
  uint8_t buf[255];

  // 注意：这里可能先读到 ACK/NAK；读到就忽略继续读
  while (true) {
    if (!read_frame_raw(wire_type, buf, L, sizeof(buf))) return false;

    if (wire_type == MSG_ACK || wire_type == MSG_NAK) {
      // 可靠层内部自己等 ACK；这里收到了就忽略
      continue;
    }

    uint8_t seq  = seq_from(wire_type);
    uint8_t btyp = base_from(wire_type);

    // 去重判断
    if (seq != s_expected_rx_seq) {
      // 说明是重复帧：上次 ACK 丢了。回 ACK，但不上交数据
      send_ack(seq);
      return false; // 本次不交付
    }

    // 新帧：回 ACK 并交付
    send_ack(seq);
    base_type = btyp;
    len = L;
    if (payload && max_len >= L && L) memcpy(payload, buf, L);
    s_expected_rx_seq ^= 1; // 更新接收期望序号
    return true;
  }
}

// ===================== 策略开关与类型判断 =====================
static uint8_t s_reliable_policy = 1; // 默认：仅 MSC 可靠
void set_reliable_policy(uint8_t policy) { s_reliable_policy = policy; }

static inline bool is_msc_type(uint8_t t) {
  uint8_t base = (uint8_t)(t & 0x7F);
  return (base == MSG_MSC_CMD || base == MSG_MSC_DATA || base == MSG_MSC_STATUS);
}

// ===================== 对外统一接口（原名不变） =====================

bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  bool use_rel = (s_reliable_policy == 2) || (s_reliable_policy == 1 && is_msc_type(type));
  if (use_rel) return send_frame_reliable((uint8_t)(type & 0x7F), payload, len);
  return send_frame_raw(type, payload, len);
}

bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  if (s_reliable_policy == 2) {
    uint8_t base;
    if (!read_frame_reliable(base, payload, len, max_len)) return false;
    type = base;
    return true;
  } else if (s_reliable_policy == 1) {
    // 优先尝试可靠收（给 MSC 用）；如果只是 ACK/NAK 或没有可靠帧，则回落裸帧
    uint8_t base;
    if (read_frame_reliable(base, payload, len, max_len)) { type = base; return true; }
    return read_frame_raw(type, payload, len, max_len);
  } else {
    return read_frame_raw(type, payload, len, max_len);
  }
}

// ===================== 便捷：鼠标/键盘（按策略自动走相应通道） =====================
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn) {
  uint8_t p[3] = { dx, dy, btn };
  return send_frame(MSG_MOUSE, p, 3);
}

bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]) {
  uint8_t p[8];
  p[0] = modifier;
  p[1] = 0x00;             // reserved
  memcpy(&p[2], keys, 6);  // key1..key6
  return send_frame(MSG_KEYBOARD, p, 8);
}

bool readMouse(uint8_t& dx, uint8_t& dy, uint8_t& btn) {
  uint8_t type, len, p[8];
  if (!read_frame(type, p, len, sizeof(p))) return false;
  if (type != MSG_MOUSE || len < 3) return false;
  dx = p[0]; dy = p[1]; btn = p[2];
  return true;
}

bool readKeyboard(uint8_t& modifier, uint8_t keys_out[6]) {
  uint8_t type, len, p[8];
  if (!read_frame(type, p, len, sizeof(p))) return false;
  if (type != MSG_KEYBOARD || len != 8) return false;
  modifier = p[0];
  memcpy(keys_out, &p[2], 6);
  return true;
}

// ===== 兼容旧接口（原来只会发/收“鼠标3B”）=====
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn) {
  (void)sendMouseReport(dx, dy, btn);
}
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn) {
  return readMouse(dx, dy, btn);
}
