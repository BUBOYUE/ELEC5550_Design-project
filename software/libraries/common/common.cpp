#include "common.h"
#include <string.h>

//UART CRC
void uart_init(void) {
    // 配置 UART 波特率、引脚
}

void uart_send_frame(const DataFrame *frame) {
    // 打包帧并发送
}

bool uart_receive_frame(DataFrame *frame) {
    // 接收并解析完整帧，CRC 校验
    return false;
}

uint16_t calc_crc16(const uint8_t *data, size_t length) {
    // CRC16 实现
    return 0;
}