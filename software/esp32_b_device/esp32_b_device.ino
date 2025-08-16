#include "device.h"
#include "uart.h"
#include <Arduino.h>

void setup() {
  device_usb_init();                            // 把 S3 枚举成 USB HID 鼠标
  Serial.begin(115200);                           //串口初始化
  uart2_init();                                    // Uart初始化
  Serial.printf("setup done");
}

void loop() {
  device_poll_and_forward();                    // 串口 → HID 转发
}