#include "CRC.h"
#include <string.h>

static constexpr uint8_t MAX_PAYLOAD = 32;

// ===  CRC ===
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len)
{
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

// === 初始化 ===
void UART_begin() {
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
}

// === 发送，名字和打包 ===
// 帧: [STX][LEN][PAYLOAD..][CRC_H][CRC_L]，PAYLOAD=3字节{dx,dy,btn}
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn)
{
  uint8_t payload[3] = {dx, dy, btn};
  uint8_t len = sizeof(payload);

  uint8_t buf[2 + sizeof(payload) + 2]; // STX + LEN + payload + CRC(2)
  buf[0] = STX;
  buf[1] = len;
  memcpy(&buf[2], payload, len);

  uint16_t crc = crc16_ccitt_false(payload, len);
  buf[2 + len + 0] = (crc >> 8) & 0xFF;
  buf[2 + len + 1] = (crc >> 0) & 0xFF;

  Serial2.write(buf, 2 + len + 2);
}

// === 接收状态机 ===
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn) {
  enum { WAIT_STX, WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC_H, WAIT_CRC_L };
  static uint8_t state = WAIT_STX;
  static uint8_t len = 0, idx = 0;
  static uint8_t payload[MAX_PAYLOAD];
  static uint8_t crc_h = 0;

  while (Serial2.available()) {
    uint8_t c = (uint8_t)Serial2.read();
    switch (state) {
      case WAIT_STX:
        if (c == STX) state = WAIT_LEN;
        break;
      case WAIT_LEN:
        len = c;
        if (len == 0 || len > MAX_PAYLOAD) { state = WAIT_STX; break; }
        idx = 0;
        state = WAIT_PAYLOAD;
        break;
      case WAIT_PAYLOAD:
        payload[idx++] = c;
        if (idx >= len) state = WAIT_CRC_H;
        break;
      case WAIT_CRC_H:
        crc_h = c;
        state = WAIT_CRC_L;
        break;
      case WAIT_CRC_L: {
        uint16_t crc_rx = ((uint16_t)crc_h << 8) | c;
        uint16_t crc_calc = crc16_ccitt_false(payload, len);
        state = WAIT_STX; // 重置状态机

        if (crc_rx == crc_calc) {
          if (len >= 3) { dx = payload[0]; dy = payload[1]; btn = payload[2]; }
          else { dx = dy = btn = 0; }
          return true;
        }
        break;
      }
    }
  }
  return false;
}
