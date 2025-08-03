#ifndef HOST_H
#define HOST_H

#include "common.h"

void usb_host_init(void);
bool usb_host_get_mouse_event(MouseEvent *event);
bool usb_host_get_keyboard_event(KeyboardEvent *event);

#endif