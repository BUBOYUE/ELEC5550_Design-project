#include "device.h"
#include "uart.h"
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
#include "tusb.h"   // 直接用 TinyUSB 的底层 API 发送键盘报告


// ===== 全局对象：HID 总线 + 设备 =====
static USBHID             g_hid;            // HID 总线
static USBHIDRelativeMouse g_mouse;         // 相对坐标鼠标（默认构造）
static USBHIDKeyboard      g_keyboard;      // 键盘（默认构造）
static bool           g_usb_ready = false;  //初始化前认为鼠标还没准备好

// ================= 复合 HID 初始化（键盘 + 鼠标）=================
void device_usb_init() {
  // 1) 原生 USB 启动
  USB.begin();

  // 2) 将键盘/鼠标都挂到同一个 HID 复合设备上
  g_hid.begin();
  g_mouse.begin();
  g_keyboard.begin();

  g_usb_ready = true;
  // 可选：给点时间让主机完成枚举
  delay(50);
}

// ================= 主循环：从 UART 读帧 → 分发到 HID =================
void device_poll_and_forward() {
  if (!g_usb_ready) { delay(1); return; }

  uint8_t type = 0, len = 0;
  uint8_t payload[8]; // 够装 3B 鼠标或 8B 键盘

  // 把串口里累积的帧尽量吃干净
  while (read_frame(type, payload, len, sizeof(payload))) {
    
    // --- 鼠标：type = 0x01, payload = [buttons, dx, dy, wheel] (4B)---
      if (type == MSG_MOUSE && len >= 4) {     // 确保 MSG_MOUSE == 0x01
      uint8_t btn = payload[0];              // bit0 L, bit1 R, bit2 M
      int8_t  dx  = (int8_t)payload[1];      // 已是有符号位移
      int8_t  dy  = (int8_t)payload[2];
      int8_t  wheel = (int8_t)payload[3];    // 垂直滚轮（无则发0）

      g_mouse.buttons(btn);
      g_mouse.move(dx, dy, wheel, 0 /*hWheel*/);
      Serial.printf("mouse btn=%02X dx=%d dy=%d wheel=%d\n", btn, dx, dy, wheel); //调试用，打印滚轮值
    }

    // --- 键盘：payload = [mod, reserved, key1..key6] (Boot 6KRO) ---
    else if (type == MSG_KEYBOARD && len == 8) {
      const uint8_t mod  = payload[0];
      const uint8_t *keys = &payload[2];   // 6 个 HID usage ID

      // 组装 8 字节键盘报告
      uint8_t report[8];
      report[0] = mod;
      report[1] = 0;
      memcpy(&report[2], keys, 6);

      // ★ 关键：用“键盘的 Report-ID”发送，而不是 0（Boot）
      //Report ID: Mouse=2,Keyboard=1
      const uint8_t REPORT_ID_KEYBOARD = 1;

      // 发送：带 Report-ID 的通用接口
      tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(report));


      //调试（可留可删）
      Serial.printf("kbd mod=%02X keys=%02X %02X %02X %02X %02X %02X\n", mod, keys[0],keys[1],keys[2],keys[3],keys[4],keys[5]);
    }

    // 可选调试
    // Serial.printf("RX type=0x%02X len=%u\n", type, len);
  }

  // 让出时间片，避免看门狗
  delay(1);
}
