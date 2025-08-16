#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stddef.h>

// 数据帧类型
#define FRAME_TYPE_MOUSE    0x01
#define FRAME_TYPE_KEYBOARD 0x02

// 鼠标事件
typedef struct {
    int8_t dx;
    int8_t dy;
    uint8_t buttons;
} MouseEvent;

// 键盘事件
typedef struct {
    uint8_t keycode[6];     // 最多6键
    uint8_t modifiers;      // Ctrl/Shift/Alt等
} KeyboardEvent;

// 帧格式
typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t payload[60];
    uint16_t crc;
} DataFrame;



#endif