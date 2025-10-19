/*
 * Project: ELEC5550 Laser USB Passthrough (ESP32-S3)
 * File:    device.h
 *
 * Author:  Elyney OU
 * Institution: University of Western Australia (UWA)
 * Course:  ELEC5550 Design Project – Group 13
 *
 * Summary:
 *   This header file defines the USB Device-side interface for the optical
 *   USB bridging system. The ESP32-S3 board functions as a composite USB
 *   device (HID + MSC), interacting with the Host-side ESP32-S3 board via
 *   UART-based frame transmission. It supports HID mouse/keyboard reports
 *   and MSC sector read/write handling, providing transparent passthrough
 *   to the PC.
 *
 * Functional Overview:
 *   - USB Device initialization (TinyUSB composite HID + MSC)
 *   - UART communication for inter-board frame exchange
 *   - MSC SCSI command handling (READ10 / WRITE10)
 *   - HID report forwarding (keyboard/mouse)
 *   - Automatic mode switching and reinitialization
 *
 * Reference:
 *   - Based on ESP-IDF and Arduino-ESP32 examples:
 *       https://github.com/espressif/arduino-esp32/blob/master/libraries/USB/examples/USBMSC/USBMSC.ino
 *       https://github.com/espressif/esp-idf/blob/master/examples/peripherals/usb/device/tusb_hid/main/tusb_hid_example_main.c
 *   - TinyUSB stack: https://github.com/hathach/tinyusb
 *
 * Build Notes:
 *   - Platform: Arduino-ESP32 (ESP32-S3)
 *   - Baud rate: 460800 bps (recommended)
 *   - Dual-board setup: Host ↔ Device via UART / optical link
 *
 * Disclaimer:
 *   Developed by UWA students for academic purposes. Provided "as is"
 *   without any warranty or guarantee of performance.
 */

#pragma once
#include <stdint.h>

/**
 * @brief USB Device operating modes
 * 
 * Defines the current operational state of the USB device:
 * - HID: Operating as HID composite device (mouse + keyboard)
 * - USTICK: Operating as USB Mass Storage device (flash drive)
 * - NONE: Uninitialized state, waiting for mode determination
 */
enum {HID, USTICK, NONE};

/**
 * @brief Current device operating mode
 * 
 * Global variable indicating which USB device class is currently active.
 * Modified during runtime based on received UART frames from host board.
 */
extern int g_Mode;

/**
 * @brief USB device readiness flag
 * 
 * Indicates whether USB device initialization has completed successfully
 * and the device is ready to handle USB requests from the host PC.
 */
extern bool g_usb_ready;

/**
 * @brief Initialize USB device based on host requirements
 * 
 * Waits for initialization frame from host board via UART, then configures
 * the ESP32-S3 as either HID composite device or MSC device accordingly.
 * Handles timeout scenarios and automatic fallback to HID mode.
 */
void device_usb_init();

/**
 * @brief Initialize UART communication with host board
 * 
 * @param rxPin GPIO pin number for UART receive
 * @param baud Baud rate for UART communication
 * 
 * Configures UART interface for inter-board communication with the
 * host ESP32-S3 board. Used for frame-based protocol exchange.
 */
void device_uart_init(int rxPin, unsigned long baud);

/**
 * @brief Main HID device loop
 * 
 * Continuously processes incoming UART frames containing HID reports
 * (mouse movements, button clicks, keyboard presses) and forwards them
 * to the connected PC via USB HID interface. Handles mode switching
 * when MSC initialization frames are received.
 */
void device_mouse_and_keyboard();

/**
 * @brief Legacy UART initialization function
 * 
 * @deprecated Use device_uart_init() instead
 * Basic UART setup function, kept for backward compatibility.
 */
void uart_init();

/**
 * @brief Check if device reinitialization is required
 * 
 * @return true if reinitialization needed, false otherwise
 * 
 * Monitors incoming UART frames for device removal notifications or
 * mode change requests from the host board. Used to detect when
 * USB devices are disconnected and reinit is necessary.
 */
bool check_reinit_needed();

/**
 * @brief Cleanly shutdown USB Mass Storage device
 * 
 * Properly terminates MSC operations, resets global state variables,
 * and prepares the system for reinitialization. Called when USB
 * storage device is removed or mode switching is required.
 */
void Close_Ustick();
