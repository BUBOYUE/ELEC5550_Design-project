#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// 帧格式: [0xAA][LEN][PAYLOAD..][CRC_H][CRC_L]
// 这里的 PAYLOAD 演示为 3字节: dx, dy, btn
const uint8_t STX = 0xAA;

// 与 uart.cpp 保持一致（ESP32 Arduino 的 Serial2 需显式指定引脚）
static const int PIN_RX = 18;   
static const int PIN_TX = 17;  
static const unsigned long BAUD = 115200;



void uart2_init();
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif