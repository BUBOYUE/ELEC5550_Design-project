#include "device.h"
#include "uart.h"
#include <Arduino.h>

// ===== 项目参数 =====
static const int      UART_RX_PIN = 18;        // 从 ESP32A 接收的 RX 脚
static const int      UART_TX_PIN = 17;        // 从 ESP32A 发送的 TX 脚
static const uint32_t UART_BAUD   = 115200;    // 与 A 端保持一致

void setup() {
  //device_usb_init();                            // 把 S3 枚举成 USB HID 鼠标
  Serial.begin(115200);                           //串口初始化
  uart_init();                                    // Uart初始化
  Serial.printf("setup done");
}

void loop() {
  device_poll_and_forward();                    // 串口 → HID 转发
}