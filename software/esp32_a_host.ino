#include <common.h>          // UART + CRC
#include <host.h>    // USB Host（鼠标/键盘）

void setup() {
    usb_host_init();   // 初始化 USB Host（读取鼠标/键盘）
    uart_init();       // 初始化 UART
}

void loop() {
    MouseEvent me;
    KeyboardEvent ke;
    DataFrame frame;

    // 读取鼠标事件并通过 UART 发送
    if (usb_host_get_mouse_event(&me)) {
        frame.type = FRAME_TYPE_MOUSE;
        frame.length = sizeof(me);
        memcpy(frame.payload, &me, sizeof(me));
        frame.crc = calc_crc16((uint8_t*)&frame, frame.length + 2);
        uart_send_frame(&frame);
    }

    // 读取键盘事件并通过 UART 发送
    if (usb_host_get_keyboard_event(&ke)) {
        frame.type = FRAME_TYPE_KEYBOARD;
        frame.length = sizeof(ke);
        memcpy(frame.payload, &ke, sizeof(ke));
        frame.crc = calc_crc16((uint8_t*)&frame, frame.length + 2);
        uart_send_frame(&frame);
    }
}