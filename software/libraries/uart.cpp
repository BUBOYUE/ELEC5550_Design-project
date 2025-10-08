#include <Arduino.h>
#include <string.h>
#include "uart.h"

// ===== 简单错误打印开关（只在丢帧时打一行）=====
#ifndef UART_PRINT_ERRORS
#define UART_PRINT_ERRORS 1   // 改成 0 可关闭所有错误打印
#endif
#define UART_ERR(fmt, ...) do { if (UART_PRINT_ERRORS) Serial.printf("[UART][DROP] " fmt, ##__VA_ARGS__); } while(0)

// ===== 引入 Reed–Solomon 头文件（mersinvald/rs.hpp）=====
#include "rs.hpp"
// RS(255,239) => 冗余16B，可纠8字节错误
using FEC_RS = RS::ReedSolomon<239, 16>;

// ===== UART2 初始化 =====
void uart2_init() {
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(50);
}

// ===== CRC16-CCITT-FALSE =====
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  const uint16_t poly = 0x1021;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ poly;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// ——开关：除了键鼠都走FEC——
static inline bool fec_enabled_for(uint8_t type) {
  switch (type) {
    case MSG_MOUSE:
    case MSG_KEYBOARD:
      return false;      // 键鼠不走FEC（低时延小包）
    default:
      return true;       // 其他一律走FEC（控制/数据/自定义类型等）
  }
}

// ===== 低层：裸帧发送 =====
// 帧格式: [STX][TYPE][LEN][PAYLOAD..][CRC_H][CRC_L]
// 外层CRC覆盖 TYPE|LEN|PAYLOAD
bool send_frame_nofec(uint8_t type, const uint8_t* payload, uint8_t len) {
  // 发送缓冲：STX(1)+TYPE(1)+LEN(1)+PAYLOAD(≤255)+CRC(2)
  static uint8_t buf[3 + 255 + 2];
  size_t off = 0;
  buf[off++] = STX;
  buf[off++] = type;
  buf[off++] = len;
  if (len && payload) {
    memcpy(&buf[off], payload, len);
    off += len;
  }
  // 计算外层CRC（TYPE|LEN|PAYLOAD）
  uint16_t crc2 = crc16_ccitt_false(&buf[1], 2 + len);
  buf[off++] = (uint8_t)(crc2 >> 8);
  buf[off++] = (uint8_t)(crc2 & 0xFF);

  size_t wrote = Serial2.write(buf, off);
  return (wrote == off);
}

// ===== 低层：裸帧接收（非阻塞状态机；循环调用直到返回true）=====
bool read_frame_nofec(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  enum { WAIT_STX, WAIT_TYPE, WAIT_LEN, READ_PAYLOAD, READ_CRC_H, READ_CRC_L };
  static uint8_t  state = WAIT_STX;
  static uint8_t  t = 0;
  static uint8_t  L = 0;
  static uint8_t  pbuf[255];
  static uint8_t  pi = 0;
  static uint16_t crc_recv = 0;
  static uint32_t last_ms = 0;

  auto reset = [&](){
    state = WAIT_STX; t = 0; L = 0; pi = 0; crc_recv = 0; last_ms = millis();
  };

  if (last_ms == 0) last_ms = millis();
  // 超时复位
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
        if (L == 0)  { state = READ_CRC_H; }
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
        // 校验 CRC（TYPE|LEN|PAYLOAD）
        static uint8_t tmp[2 + 255];
        tmp[0] = t; tmp[1] = L;
        if (L) memcpy(&tmp[2], pbuf, L);
        uint16_t crc_calc = crc16_ccitt_false(tmp, 2 + L);
        if (crc_calc == crc_recv) {
          // 输出
          type = t;
          len  = L;
          if (L) {
            if (max_len < L) {
              // 上层给的缓存不够，丢弃并打印
              UART_ERR("payload overflow (nofec): type=0x%02X len=%u max=%u\n", t, L, (unsigned)max_len);
              reset();
              return false;
            }
            memcpy(payload, pbuf, L);
          }
          reset();
          return true;
        } else {
          // CRC 错，丢弃并打印
          UART_ERR("CRC fail: type=0x%02X len=%u\n", t, L);
          reset();
        }
        break;
      }
    }
  }
  return false;
}

// ===== 顶层：统一入口（按类型自动选择是否启用FEC）=====

// 发送：FEC 类型 -> 短化 RS 变长上线；非FEC类型 -> 裸帧原样发送
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (!fec_enabled_for(type)) {
    return send_frame_nofec(type, payload, len);
  }

  // FEC 消息：[raw_len(1B) | raw_data | innerCRC(2B)] 放前部；剩余为 0
  if (len > 236) return false; // 留1B长度+2B CRC
  uint8_t kbuf[239];
  memset(kbuf, 0, sizeof(kbuf));
  kbuf[0] = len;                           // 原始长度
  if (len) memcpy(&kbuf[1], payload, len);
  uint16_t inner = crc16_ccitt_false(&kbuf[1], len);
  kbuf[1 + len] = (uint8_t)(inner >> 8);
  kbuf[2 + len] = (uint8_t)(inner & 0xFF);
  // 其余保持为 0（可短化）

  // 先得到完整 255B 码字
  uint8_t code_full[239 + 16];
  FEC_RS rs;
  rs.Encode(kbuf, code_full);

  // ——短化发送：只发“前 k' 个消息字节 + 16B 冗余”，去掉尾部全 0 的消息字节——
  const uint16_t kprime   = (uint16_t)(1 + len + 2);  // 有效消息字节
  const uint16_t wire_len = (uint16_t)(kprime + 16);  // 上线长度（变长：19..255）
  uint8_t wire[239 + 16];
  memcpy(wire,        code_full,        kprime);      // 消息有效部分
  memcpy(wire+kprime, code_full + 239,  16);          // 末尾 16B 冗余

  return send_frame_nofec(type, wire, (uint8_t)wire_len);
}

// 接收：先按裸帧取回；若FEC类型 -> 按 LEN 重建 255B 码字再解码
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len) {
  uint8_t t, L;
  uint8_t buf[255];
  if (!read_frame_nofec(t, buf, L, sizeof(buf))) return false;

  if (!fec_enabled_for(t)) {
    // 非FEC类型，直接透传
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

  // FEC类型：支持“变长短化”——允许 19..255
  if (L < (1 + 2 + 16) || L > (239 + 16)) {
    UART_ERR("FEC len out of range: type=0x%02X len=%u (expect 19..255)\n", t, L);
    return false;
  }

  const uint16_t kprime = (uint16_t)(L - 16); // 本帧携带的消息有效字节
  // 重建完整 255B 码字：[239B 消息 | 16B 冗余]
  uint8_t code_full[239 + 16];
  if (kprime < 239) {
    memcpy(code_full,         buf,         kprime);         // 有效消息
    memset(code_full+kprime,  0,           239 - kprime);   // 省略部分补 0
    memcpy(code_full + 239,   buf + kprime, 16);            // 16B 冗余
  } else {
    // 满长 255
    memcpy(code_full, buf, 239 + 16);
  }

  // RS 解码：255 -> 239
  uint8_t kbuf[239];
  FEC_RS rs;
  int dec_ret = rs.Decode(code_full, kbuf, nullptr, 0); // >=0 成功；<0 失败
  if (dec_ret < 0) {
    UART_ERR("FEC decode fail: type=0x%02X\n", t);
    return false;
  }

  // 解析出 [raw_len | raw_data | innerCRC]
  uint8_t raw_len = kbuf[0];
  if (raw_len > 236) {
    UART_ERR("FEC raw_len illegal: %u\n", raw_len);
    return false;
  }
  if ((size_t)raw_len > max_len) {
    UART_ERR("payload overflow (fec->upper): raw_len=%u max=%u\n", raw_len, (unsigned)max_len);
    return false;
  }

  uint16_t inner_got  = ((uint16_t)kbuf[1 + raw_len] << 8) | kbuf[2 + raw_len];
  uint16_t inner_calc = crc16_ccitt_false(&kbuf[1], raw_len);
  if (inner_got != inner_calc) {
    UART_ERR("FEC inner CRC fail: raw_len=%u\n", raw_len);
    return false;
  }

  if (raw_len) memcpy(payload, &kbuf[1], raw_len);
  type = t;
  len  = raw_len;
  // 成功时不打印，保持安静
  return true;
}

// ===== 便捷包装：鼠标/键盘 =====
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn) {
  uint8_t p[3] = { dx, dy, btn };
  return send_frame(MSG_MOUSE, p, 3);
}
bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]) {
  uint8_t p[8] = { modifier, 0, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5] };
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
