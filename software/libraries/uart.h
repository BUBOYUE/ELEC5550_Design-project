#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// ===== 帧头常量 =====
static const uint8_t STX = 0xAA;

// ===== 帧类型（自定义，可扩展）=====
enum MsgType : uint8_t {
  MSG_MOUSE    = 0x01, // payload: 3B -> dx, dy, btn
  MSG_KEYBOARD = 0x02, // payload: 8B -> modifier, reserved, key1..key6
  // 预留：
  MSG_SET_MODE = 0x20, // 可选：若将来需要
  MSG_ACK      = 0x7E,
  MSG_NAK      = 0x7F,
};

// ===== 串口参数（与你现有工程保持一致）=====
static const int PIN_RX = 18;
static const int PIN_TX = 17;
static const unsigned long BAUD = 115200;

// ===== API =====
void uart2_init();

// 计算 CRC16-CCITT-FALSE（多态：计算任意缓冲）
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

// 低层通用帧发送/接收（推荐优先使用）
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len);
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// 便捷包装：鼠标/键盘
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn);
bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]);

// （可选）便捷读取包装：若你想直接按类型拿到解析结果
bool readMouse(uint8_t& dx, uint8_t& dy, uint8_t& btn);
bool readKeyboard(uint8_t& modifier, uint8_t keys_out[6]);

// ===== 兼容你原有的旧接口（不改上层现有调用也能跑）=====
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif
