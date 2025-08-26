#include <Arduino.h>
#include <string.h>
#include "uart.h"

// ===== UART2 初始化 =====
void uart2_init() {
  // ESP32 的 Serial2: (baud, config, RX, TX)
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(50);
}

// ===== CRC16-CCITT-FALSE =====
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

// 计算覆盖 TYPE|LEN|PAYLOAD 的 CRC，便于调用
static uint16_t crc_type_len_payload(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t tmp[2 + 255]; // LEN 最大255
  tmp[0] = type;
  tmp[1] = len;
  if (len && payload) memcpy(&tmp[2], payload, len);
  return crc16_ccitt_false(tmp, 2 + len);
}

// ===== 通用帧发送 =====
// 帧格式: [STX][TYPE][LEN][PAYLOAD..][CRC_H][CRC_L]
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
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

// ===== 通用帧接收（状态机）=====
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
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
        if (s_len == 0) { // 允许0长度有效载荷
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

        state = WAIT_STX; // 重置状态机以继续接收下一帧

        if (crc_rx == crc_calc) {
          // 输出
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

// ===== 便捷：鼠标/键盘 =====
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn) {
  uint8_t p[3] = { dx, dy, btn }; // 与你们现有顺序一致
  return send_frame(MSG_MOUSE, p, 3);
}

bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]) {
  uint8_t p[8];
  p[0] = modifier;
  p[1] = 0x00;        // reserved
  memcpy(&p[2], keys, 6); // key1..key6
  return send_frame(MSG_KEYBOARD, p, 8);
}

// 可选：读取便捷包装
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
  // p[1] 是 reserved
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
