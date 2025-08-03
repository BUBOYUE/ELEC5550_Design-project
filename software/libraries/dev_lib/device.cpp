#include "device.h"

void usb_device_init(void) {
    // 初始化 TinyUSB 作为 HID Device
}

void usb_device_send_mouse(const MouseEvent *event) {
    // 发送鼠标 HID 报告给电脑
}

void usb_device_send_keyboard(const KeyboardEvent *event) {
    // 发送键盘 HID 报告给电脑
}