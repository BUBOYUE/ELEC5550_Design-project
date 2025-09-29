#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// ===== 帧头常量 =====
static const uint8_t STX = 0xAA;

// ===== 帧类型（自定义，可扩展）=====
enum MsgType : uint8_t {
  // HID
  MSG_MOUSE      = 0x01, // payload: 3B -> dx, dy, btn
  MSG_KEYBOARD   = 0x02, // payload: 8B -> modifier, reserved, key1..key6

  // 预留：通用设置
  MSG_SET_MODE   = 0x20,

  // Mass Storage Class (U盘)——建议用这三类来承载 CBW/DATA/CSW
  MSG_MSC_CMD    = 0x30, // CBW / SCSI command
  MSG_MSC_DATA   = 0x31, // data in/out
  MSG_MSC_STATUS = 0x32, // CSW / status

  // 可靠层控制
  MSG_ACK        = 0x7E,
  MSG_NAK        = 0x7F,
};

// ===== 串口参数（与你现有工程保持一致）=====
static const int PIN_RX = 18;
static const int PIN_TX = 17;
static const unsigned long BAUD = 115200;

// ===== API =====
void uart2_init();

// CRC16-CCITT-FALSE
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

// 全局可靠性策略：0=关闭；1=仅MSC可靠（默认）；2=全部类型可靠
void set_reliable_policy(uint8_t policy);

// —— 对外统一接口（内部按策略自动选择 裸帧 or 可靠帧）——
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len);
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// 便捷包装：鼠标/键盘（仍然调用 send_frame，按策略自动走可靠或裸帧）
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn);
bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]);

// （可选）便捷读取包装
bool readMouse(uint8_t& dx, uint8_t& dy, uint8_t& btn);
bool readKeyboard(uint8_t& modifier, uint8_t keys_out[6]);

// ===== 兼容你原有的旧接口（不改上层现有调用也能跑）=====
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif
