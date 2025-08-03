#include <common.h>           // UART + CRC
#include <device.h>   // USB HID Device（模拟鼠标/键盘）

void setup() {
    uart_init();        // 初始化 UART
    usb_device_init();  // 初始化 USB HID（电脑识别为输入设备）
}

void loop() {
    DataFrame frame;

    // 从 UART 接收帧并转发到电脑
    if (uart_receive_frame(&frame)) {
        if (frame.type == FRAME_TYPE_MOUSE) {
            MouseEvent *m = (MouseEvent*)frame.payload;
            usb_device_send_mouse(m);
        } 
        else if (frame.type == FRAME_TYPE_KEYBOARD) {
            KeyboardEvent *k = (KeyboardEvent*)frame.payload;
            usb_device_send_keyboard(k);
        }
    }
}