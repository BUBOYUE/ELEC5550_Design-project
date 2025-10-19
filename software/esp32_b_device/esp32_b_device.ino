/*
 * Project: ELEC5550 Laser USB Passthrough (ESP32‑S3)
 * File:    esp32_b_device.ino
 *
 * Author:  Elyney OU
 * Institution: University of Western Australia (UWA)
 * Course:  ELEC5550 Design Project – Group 13
 *
 * Summary:
 *   This sketch implements the Device‑side logic of the optical USB bridging system.
 *   The ESP32‑S3 acts as a USB Device endpoint, supporting both HID (keyboard/mouse)
 *   and MSC (USB mass storage) passthrough modes. It interfaces with the Host‑side
 *   board through a custom UART frame protocol featuring CRC and optional FEC for
 *   data integrity.
 *
 * Functional Overview:
 *   - Initializes UART2 for bidirectional data exchange.
 *   - Enumerates as either HID or MSC device using TinyUSB stack.
 *   - For HID mode: forwards UART input reports to the PC in real time.
 *   - For MSC mode: coordinates SCSI command handling and re‑initialization.
 *
 * Reference:
 *   Based on and adapted from ESP‑IDF examples:
 *     - HID Device example: examples/peripherals/usb/device/hid
 *     - MSC Device example: examples/peripherals/usb/device/msc
 *   Original extensions include optical‑link data framing and automatic re‑init logic.
 *
 * Build Notes:
 *   - Platform: Arduino‑ESP32 (ESP32‑S3)
 *   - Baud rate: 460800 bps (recommended)
 *   - Switch configuration: left = A‑end, right = B‑end
 *
 * Disclaimer:
 *   Developed by UWA students team for academic use. Provided as‑is without warranty.
 */

#include "device.h"
#include "uart.h"
#include <Arduino.h>

void setup() {
  Serial.begin(921600/2);                         // Serial port initialization
  uart2_init();                                   // UART initialization
  Serial.println("Uart done");
  device_usb_init();                              // Enumerate as HID mouse/keyboard or USB storage
  Serial.println("USB initialization done");
}

void loop() {
  // if (Serial2.available()) {
  //   Serial.write(Serial2.read());}
  
  // Not initialized or exited initialization, go initialize
  if (!g_usb_ready || g_Mode == NONE) {
    device_usb_init();
    return; // Let next loop iteration judge again
  }
  
  // HID mode continuous loop until exit (forced exit/switch to USB storage)
  if (g_Mode == HID){
  device_mouse_and_keyboard();                    // UART → HID forwarding
  }

  // MSC mode does nothing, PC will call callback functions
  else if (g_Mode == USTICK){
    
    // Do nothing but detect if USB stick removed
    if (check_reinit_needed()) {
      Close_Ustick();
    }
  }

}