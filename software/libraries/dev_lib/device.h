#ifndef DEVICE_H
#define DEVICE_H

#include "common.h"

void usb_device_init(void);
void usb_device_send_mouse(const MouseEvent *event);
void usb_device_send_keyboard(const KeyboardEvent *event);

#endif