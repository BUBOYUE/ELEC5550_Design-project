#include "device.h"
#include "uart.h"
#include <Arduino.h>

void setup() {
  Serial.begin(921600/2);                         // 串口初始化
  uart2_init();                                 // Uart初始化
  Serial.println("Uart done");
  device_usb_init();                            // 枚举成 HID 鼠标键盘 or U盘
  Serial.println("USB initialization done");
}

void loop() {
  // if (Serial2.available()) {
  //   Serial.write(Serial2.read());}
  
  // 还没初始化或退出初始化了，去初始化
  if (!g_usb_ready || g_Mode == NONE) {
    device_usb_init();
    return; // 让下一轮 loop 再判断
  }
  
  // HID模式持续循环，直到跳出（强制跳出/切换为U盘）
  if (g_Mode == HID){
  device_mouse_and_keyboard();                    // 串口 → HID 转发
  }

  // MSC模式下什么也不用做，电脑会调用CB function
  else if (g_Mode == USTICK){
    
    //Do nothing but detect if U stick removed
    if (check_reinit_needed()) {
      Close_Ustick();
    }
  }

}