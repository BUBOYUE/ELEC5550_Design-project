#pragma once
#include <stdint.h>


//Device type represent
enum {HID, USTICK, NONE};
extern int g_Mode;
extern bool g_usb_ready;

// 初始化 USB + HID 鼠标
void device_usb_init();

// 初始化 UART（仅接收）。rxPin 通常为 18，baud 与 A 端一致
void device_uart_init(int rxPin, unsigned long baud);

// 主循环调用：不停从 UART 读帧并转发到 USB HID（非阻塞）
void device_mouse_and_keyboard();

void uart_init();

//detect if U stick removed
bool check_reinit_needed();

//拔出U盘准备重新进入初始化
void Close_Ustick();
