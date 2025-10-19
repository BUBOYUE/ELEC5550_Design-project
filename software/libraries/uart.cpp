/*
 * Project: ELEC5550 Laser USB Passthrough (ESP32-S3)
 * File:    uart.c
 *
 * Author:  Kunze Chen
 * Institution: University of Western Australia (UWA)
 * Course:  ELEC5550 Design Project – Group 13
 *
 * Summary:
 *   UART communication protocol header for inter-board frame transmission.
 *   Implements a framed protocol with CRC16 error detection and optional
 *   FEC (Forward Error Correction) for reliable data exchange between
 *   Host ESP32-S3 and Device ESP32-S3 boards over UART/optical link.
 *
 * Protocol Structure:
 *   [STX(1)] [Type(1)] [Length(1)] [Payload(0-255)] [CRC16(2)]
 *   - STX: Start byte (0xAA)
 *   - Type: Message type (see MsgType enum)
 *   - Length: Payload length in bytes
 *   - Payload: Message-specific data
 *   - CRC16: CCITT-FALSE checksum for error detection
 *
 * Features:
 *   - Bidirectional frame-based communication
 *   - CRC16 error detection and correction
 *   - Optional FEC for enhanced reliability
 *   - Support for HID and MSC message types
 *   - Backward compatibility with legacy interfaces
 *
 * Hardware Configuration:
 *   - Default baud rate: 460800 bps
 *   - Default RX pin: GPIO 16
 *   - Default TX pin: GPIO 18
 *   - UART2 interface used for inter-board communication
 *
 * Build Notes:
 *   - Compatible with Arduino-ESP32 framework
 *   - Requires ESP32-S3 with multiple UART support
 *   - Can be adapted for optical communication links
 */


#include <Arduino.h>
#include <string.h>
#include "uart.h"

// Error logging control
#ifndef UART_PRINT_ERRORS
#define UART_PRINT_ERRORS 1
#endif
#define UART_ERR(fmt, ...) do { if (UART_PRINT_ERRORS) Serial.printf("[UART][DROP] " fmt, ##__VA_ARGS__); } while(0)

// Reed-Solomon FEC implementation
#include "rs.hpp"
using FEC_RS = RS::ReedSolomon<239, 16>;  // RS(255,239): 16 bytes redundancy, corrects up to 8 byte errors

void uart2_init() {
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(50);  // Hardware stabilization
}

// CRC16-CCITT-FALSE calculation
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;                    // CCITT-FALSE initial value
  const uint16_t poly = 0x1021;             // CCITT polynomial
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ poly;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// FEC policy: Mouse/Keyboard use raw frames (low latency), others use FEC (high reliability)
static inline bool fec_enabled_for(uint8_t type) {
  switch (type) {
    case MSG_MOUSE:
    case MSG_KEYBOARD:
      return false;      // HID: low latency
    default:
      return true;       // Control/Data: high reliability
  }
}

// Send raw frame: [STX][TYPE][LEN][PAYLOAD][CRC16]
bool send_frame_nofec(uint8_t type, const uint8_t* payload, uint8_t len) {
  static uint8_t buf[3 + 255 + 2];
  size_t off = 0;
  buf[off++] = STX;                         // Frame delimiter
  buf[off++] = type;                        // Message type
  buf[off++] = len;                         // Payload length
  if (len && payload) {
    memcpy(&buf[off], payload, len);
    off += len;
  }
  // CRC over TYPE|LEN|PAYLOAD
  uint16_t crc2 = crc16_ccitt_false(&buf[1], 2 + len);
  buf[off++] = (uint8_t)(crc2 >> 8);        // CRC high byte
  buf[off++] = (uint8_t)(crc2 & 0xFF);      // CRC low byte

  size_t wrote = Serial2.write(buf, off);
  return (wrote == off);
}

// Receive raw frame using non-blocking state machine
bool read_frame_nofec(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  enum { WAIT_STX, WAIT_TYPE, WAIT_LEN, READ_PAYLOAD, READ_CRC_H, READ_CRC_L };
  static uint8_t  state = WAIT_STX;
  static uint8_t  t = 0;
  static uint8_t  L = 0;
  static uint8_t  pbuf[255];
  static uint8_t  pi = 0;
  static uint16_t crc_recv = 0;
  static uint32_t last_ms = 0;

  // Reset state machine
  auto reset = [&](){
    state = WAIT_STX; t = 0; L = 0; pi = 0; crc_recv = 0; last_ms = millis();
  };

  if (last_ms == 0) last_ms = millis();
  // Timeout reset (50ms)
  if (millis() - last_ms > 50) {
    reset();
  }

  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    last_ms = millis();

    switch (state) {
      case WAIT_STX:
        if (b == STX) { state = WAIT_TYPE; }
        break;

      case WAIT_TYPE:
        t = b;
        state = WAIT_LEN;
        break;

      case WAIT_LEN:
        L = b;
        if (L == 0)  { state = READ_CRC_H; }        // No payload
        else         { state = READ_PAYLOAD; pi = 0; }
        break;

      case READ_PAYLOAD:
        pbuf[pi++] = b;
        if (pi >= L) state = READ_CRC_H;
        break;

      case READ_CRC_H:
        crc_recv = ((uint16_t)b) << 8;
        state = READ_CRC_L;
        break;

      case READ_CRC_L: {
        crc_recv |= b;
        // Verify CRC over TYPE|LEN|PAYLOAD
        static uint8_t tmp[2 + 255];
        tmp[0] = t; tmp[1] = L;
        if (L) memcpy(&tmp[2], pbuf, L);
        uint16_t crc_calc = crc16_ccitt_false(tmp, 2 + L);
        if (crc_calc == crc_recv) {
          // CRC valid - output frame
          type = t;
          len  = L;
          if (L) {
            if (max_len < L) {
              UART_ERR("payload overflow (nofec): type=0x%02X len=%u max=%u\n", t, L, (unsigned)max_len);
              reset();
              return false;
            }
            memcpy(payload, pbuf, L);
          }
          reset();
          return true;
        } else {
          UART_ERR("CRC fail: type=0x%02X len=%u\n", t, L);
          reset();
        }
        break;
      }
    }
  }
  return false;
}

// Send frame with automatic FEC selection
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (!fec_enabled_for(type)) {
    return send_frame_nofec(type, payload, len);
  }

  // FEC encoding: [raw_len(1B) | raw_data | innerCRC(2B)] + zero padding
  if (len > 236) return false;              // Reserve space for length + CRC
  uint8_t kbuf[239];
  memset(kbuf, 0, sizeof(kbuf));
  kbuf[0] = len;                            // Original length
  if (len) memcpy(&kbuf[1], payload, len);  // Original data
  uint16_t inner = crc16_ccitt_false(&kbuf[1], len);
  kbuf[1 + len] = (uint8_t)(inner >> 8);   // Inner CRC
  kbuf[2 + len] = (uint8_t)(inner & 0xFF);

  // Generate full 255-byte RS codeword
  uint8_t code_full[239 + 16];
  FEC_RS rs;
  rs.Encode(kbuf, code_full);

  // Shortened transmission: send only effective message bytes + 16B redundancy
  const uint16_t kprime   = (uint16_t)(1 + len + 2);  // Effective message bytes
  const uint16_t wire_len = (uint16_t)(kprime + 16);  // Wire length (19..255)
  uint8_t wire[239 + 16];
  memcpy(wire,        code_full,        kprime);      // Message part
  memcpy(wire+kprime, code_full + 239,  16);          // Redundancy part

  return send_frame_nofec(type, wire, (uint8_t)wire_len);
}

// Receive frame with automatic FEC handling
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  uint8_t t, L;
  uint8_t buf[255];
  if (!read_frame_nofec(t, buf, L, sizeof(buf))) return false;

  if (!fec_enabled_for(t)) {
    // Non-FEC type: direct passthrough
    type = t;
    len  = L;
    if (L) {
      if (max_len < L) {
        UART_ERR("payload overflow (nofec->upper): type=0x%02X len=%u max=%u\n", t, L, (unsigned)max_len);
        return false;
      }
      memcpy(payload, buf, L);
    }
    return true;
  }

  // FEC type: support variable-length shortening (19..255 bytes)
  if (L < (1 + 2 + 16) || L > (239 + 16)) {
    UART_ERR("FEC len out of range: type=0x%02X len=%u (expect 19..255)\n", t, L);
    return false;
  }

  const uint16_t kprime = (uint16_t)(L - 16); // Effective message bytes
  // Reconstruct full 255B codeword
  uint8_t code_full[239 + 16];
  if (kprime < 239) {
    memcpy(code_full,         buf,         kprime);         // Effective message
    memset(code_full+kprime,  0,           239 - kprime);   // Zero padding
    memcpy(code_full + 239,   buf + kprime, 16);            // Redundancy
  } else {
    memcpy(code_full, buf, 239 + 16);
  }

  // RS decode: 255 -> 239
  uint8_t kbuf[239];
  FEC_RS rs;
  int dec_ret = rs.Decode(code_full, kbuf, nullptr, 0);
  if (dec_ret < 0) {
    UART_ERR("FEC decode fail: type=0x%02X\n", t);
    return false;
  }

  // Parse [raw_len | raw_data | innerCRC]
  uint8_t raw_len = kbuf[0];
  if (raw_len > 236) {
    UART_ERR("FEC raw_len illegal: %u\n", raw_len);
    return false;
  }
  if ((size_t)raw_len > max_len) {
    UART_ERR("payload overflow (fec->upper): raw_len=%u max=%u\n", raw_len, (unsigned)max_len);
    return false;
  }

  // Verify inner CRC
  uint16_t inner_got  = ((uint16_t)kbuf[1 + raw_len] << 8) | kbuf[2 + raw_len];
  uint16_t inner_calc = crc16_ccitt_false(&kbuf[1], raw_len);
  if (inner_got != inner_calc) {
    UART_ERR("FEC inner CRC fail: raw_len=%u\n", raw_len);
    return false;
  }

  // Extract payload
  if (raw_len) memcpy(payload, &kbuf[1], raw_len);
  type = t;
  len  = raw_len;
  return true;
}

// Convenience wrapper functions
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn) {
  uint8_t p[3] = { dx, dy, btn };
  return send_frame(MSG_MOUSE, p, 3);
}

bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]) {
  uint8_t p[8] = { modifier, 0, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5] };
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

// Legacy compatibility functions
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn) {
  (void)sendMouseReport(dx, dy, btn);
}

bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn) {
  return readMouse(dx, dy, btn);
}
