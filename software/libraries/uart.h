#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// ====== 默认硬件参数（如外部已有定义则不覆盖）======
#ifndef BAUD
#define BAUD 460800
#endif
#ifndef PIN_RX
#define PIN_RX 16
#endif
#ifndef PIN_TX
#define PIN_TX 18
#endif

// ===== 帧头常量 =====
static const uint8_t STX = 0xAA;

// ===== 帧类型（可扩展）=====
enum MsgType : uint8_t {
  MSG_MOUSE    = 0x01, // payload: 3B -> dx, dy, btn
  MSG_KEYBOARD = 0x02, // payload: 8B -> modifier, reserved, key1..key6
  U_A2B_INIT  = 0x03, // U stick initilize info (block size, block count)
  U_B2A_WRITE, // PC write requirement
  U_A2B_WRITEDONE, // U stick send written done to PC
  U_B2A_READ, // PC Read requirement
  U_A2B_READCONTENT, // U stick send required content to PC
  U_B2A_STARTSTOP, // PC require U stick plug in/out
  U_A2B_REMOVED, // U stick Remove
  U_EMPTY, // All zero
  MSG_FEC_DATA, // FEC 载荷（开启FEC的类型）
  CMD_REINIT  = 0xff, // Reinitial command
};

// ===== 对外 API =====
void     uart2_init();

// CRC16-CCITT-FALSE（外层与内层都复用）
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

// 帧收发（对外名字保持不变；内部已集成“按类型可选FEC”）
bool     send_frame(uint8_t type, const uint8_t* payload, uint8_t len);
bool     read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// ——保留“旧实现（裸帧）”以便内部复用，不建议上层直接调用——
bool     send_frame_nofec(uint8_t type, const uint8_t* payload, uint8_t len);
bool     read_frame_nofec(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// 便捷包装：鼠标/键盘（保持你们现有用法不变）
bool     sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn);
bool     sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]);

// 便捷读取包装（可选）
bool     readMouse(uint8_t& dx, uint8_t& dy, uint8_t& btn);
bool     readKeyboard(uint8_t& modifier, uint8_t keys_out[6]);

// ===== 兼容你们“旧接口名”（只发/收鼠标3B）=====
void     sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool     readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif
