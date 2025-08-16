#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif