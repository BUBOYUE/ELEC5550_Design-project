#include "device.h"
#include "uart.h"
#include <Arduino.h>
#include "USB.h"
#include "USBHIDMouse.h"

// ===== 可配常量 =====
#ifndef STX
#define STX 0xA5              // 若你的协议 STX = 0x02，则在工程里先 #define STX 0x02 再编译
#endif

#ifndef UART_READ_SLICE
#define UART_READ_SLICE 0     // readFrame 内部按 available() 轮询，这里设 0 以免额外阻塞
#endif

// ===== 全局对象声明 =====
static USBHIDMouse g_mouse;
static bool g_usb_ready = false; //初始化成功前认为鼠标还没准备好

// 用 Serial2 做数据口（ESP32-S3 可多个 UART），将其命名为DATA_UART，在后面的代码中使用。如果想要更改所用的串口，只需要在该行更改。
static HardwareSerial &DATA_UART = Serial2;

// ================= USB / HID 初始化（将ESP32设为鼠标）=================
void device_usb_init() {
  USB.begin();     // ESP32-S3 走原生 USB
  g_mouse.begin(); 
  g_usb_ready = true; //标记鼠标初始化成功
}

// ================= UART 初始化 =================
void uart_init() {
  delay(10);
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);//板间通信串口，接线时要 TX ↔ 对方 RX，并且 GND ↔ GND
}


// ================= 主循环转发 =================
void device_poll_and_forward() {
  uint8_t dx_u, dy_u, btn_u;
  
  // 把串口里累积的帧尽量吃干净，逐帧上报
  while (readFrame(dx_u, dy_u, btn_u)) {
    Serial.printf("TX: dx=%u dy=%u btn=%u\n", dx_u, dy_u, btn_u);
    if (!g_usb_ready) continue;

    // 二补码解释：uint8_t -> int8_t 【？意义不明】
    int8_t dx = (int8_t)dx_u;
    int8_t dy = (int8_t)dy_u;

    // // 按钮为绝对掩码（与 Arduino HID 约定一致：bit0 L, bit1 R, bit2 M, ...）
    // g_mouse.buttons(btn_u);

    // // 没有滚轮字段时置 0
    // g_mouse.move(dx, dy, /*wheel=*/0, /*hwheel=*/0);
  }

  // 让出时间片，避免看门狗
  delay(1);
}

