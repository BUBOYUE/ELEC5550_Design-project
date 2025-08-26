#pragma once
#include <Arduino.h>

// === 原始常量 ===
#define ROLE_SENDER 1        // 仅做兼容；库不使用它
const int UART_NUM = 2;      // 使用 Serial2
const int PIN_RX   = 18;     // 按你实际连线
const int PIN_TX   = 17;     // 按你实际连线
const unsigned long BAUD = 115200;

const uint8_t STX = 0xAA;    // 帧起始标志

// 供外部调用的 API
void UART_begin();                            // 初始化 Serial2
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);