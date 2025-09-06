#include "device.h"
#include "uart.h"
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"

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

    // --- 键盘：payload = [mod, reserved, key1..key6] (6KRO+mod) ---
    else if (type == MSG_KEYBOARD && len == 8) {
      const uint8_t mod = payload[0];
      const uint8_t *keys = &payload[2]; // 6 个按键扫描码

      /*
       * 不同版本的 USBHIDKeyboard 提供的“直接发 8 字节报告”的 API 名字可能不同。
       * 优先尝试 sendReport(mod, keys[6]) 之类；如果你库里没有该方法，
       * 就退化为：releaseAll() → 按下所有修饰键和普通键。
       */
      #if defined(USBHID_KEYBOARD_HAS_SENDREPORT8)
        g_keyboard.sendReport(mod, keys);        // 如果你们库支持，优先用一发到位
      #else
        // 保险做法：先清空，再逐个“按下”
        g_keyboard.releaseAll();

        // 修饰键（bit0..bit7）：LCTL LSHIFT LALT LGUI RCTL RSHIFT RALT RGUI
        auto press_modifier = [&](uint8_t bitmask){
          switch (bitmask) {
            case 0x01: g_keyboard.press(KEY_LEFT_CTRL);  break;
            case 0x02: g_keyboard.press(KEY_LEFT_SHIFT); break;
            case 0x04: g_keyboard.press(KEY_LEFT_ALT);   break;
            case 0x08: g_keyboard.press(KEY_LEFT_GUI);   break;
            case 0x10: g_keyboard.press(KEY_RIGHT_CTRL); break;
            case 0x20: g_keyboard.press(KEY_RIGHT_SHIFT);break;
            case 0x40: g_keyboard.press(KEY_RIGHT_ALT);  break;
            case 0x80: g_keyboard.press(KEY_RIGHT_GUI);  break;
          }
        };
        for (uint8_t b = 0; b < 8; ++b) {
          if (mod & (1u << b)) press_modifier(1u << b);
        }

        // 普通 6 键（0 表示“无按键”）
        for (int i = 0; i < 6; ++i) {
          if (keys[i] != 0) g_keyboard.press(keys[i]);
        }
        // 说明：
        // 1) 多数实现中 press()/releaseAll() 会立即发送报告；
        // 2) 若你们库需要显式 flush，可在这里调用 g_keyboard.sendReport(...) 或类似 API。
      #endif
    }

    // 可选调试
    // Serial.printf("RX type=0x%02X len=%u\n", type, len);
  }

  // 让出时间片，避免看门狗
  delay(1);
}
