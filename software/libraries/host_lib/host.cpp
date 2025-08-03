#include "host.h"

void usb_host_init(void) {
    // 初始化 TinyUSB Host
    // tusb_init();
    // 挂载 HID 设备
}

bool usb_host_get_mouse_event(MouseEvent *event) {
    // 从 TinyUSB Host 读取鼠标事件
    // 如果有新事件，填充 event 并返回 true
    return false;
}

bool usb_host_get_keyboard_event(KeyboardEvent *event) {
    // 从 TinyUSB Host 读取键盘事件
    return false;
}