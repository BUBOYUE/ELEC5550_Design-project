#pragma once
#include <stdint.h>


// #define ROLE_SENDER 1   // 1=发端(A)，0=收端(B)
// const int UART_NUM = 2; // 使用 Serial2
// const int PIN_RX = 18;  // 按你实际连线改
// const int PIN_TX = 17;  // 按你实际连线改
// const unsigned long BAUD = 115200;//波特率


// 初始化 USB + HID 鼠标
void device_usb_init();

// 初始化 UART（仅接收）。rxPin 通常为 18，baud 与 A 端一致
void device_uart_init(int rxPin, unsigned long baud);

// 主循环调用：不停从 UART 读帧并转发到 USB HID（非阻塞）
void device_poll_and_forward();

void uart_init();

