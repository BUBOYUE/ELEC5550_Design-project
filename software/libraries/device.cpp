/*
 * Project: ELEC5550 Laser USB Passthrough (ESP32-S3)
 * File:    device.cpp
 *
 * Author:  Elyney OU
 * Institution: University of Western Australia (UWA)
 * Course:  ELEC5550 Design Project – Group 13
 *
 * Summary:
 *   This source file implements the USB Device-side logic of the optical
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
 *   Developed by UWA students for academic purposes. Provided “as is”
 *   without any warranty or guarantee of performance.
 */

#include "device.h"
#include "uart.h"
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
#include "USBMSC.h"
#include "tusb.h"   // Use TinyUSB low-level API to send keyboard reports

#define DEBUG 0  //debug mode


// ===== Global objects: HID bus + devices =====
static USBHID              g_hid;           // HID bus
static USBHIDRelativeMouse g_mouse;         // Relative coordinate mouse (default constructor)
static USBHIDKeyboard      g_keyboard;      // Keyboard (default constructor)
static USBMSC              MSC;             // USB stick device object
bool   g_usb_ready = false;    // Consider mouse not ready before initialization
int    g_Mode = NONE;  // Device Mode


//================== Callback functions must be written before initialization =====================

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  #ifdef DEBUG
  Serial.printf("MSC WRITE: lba: %lu, offset: %lu, bufsize: %lu\n", lba, offset, bufsize);
    #endif
    const uint16_t MAX_PAYLOAD = 224;   // Maximum single frame data payload
    uint32_t sent = 0; // Bytes sent

    // == Send B board WRITE request ==
    while (sent < bufsize) {
        // Data length for this frame
        uint32_t chunk = bufsize - sent;
        if (chunk > MAX_PAYLOAD) chunk = MAX_PAYLOAD;

        // Calculate absolute sector position for this frame
        uint32_t absolute  = offset + sent;
        uint32_t frame_lba = lba + (absolute / 512);   // Auto-increment when crossing sectors
        uint32_t frame_off = absolute % 512;

        // Assemble payload structure
        struct {
            uint32_t lba;
            uint32_t offset;
            uint32_t len;
            uint8_t  data[MAX_PAYLOAD];
        } __attribute__((packed)) payload;

        payload.lba    = frame_lba;
        payload.offset = frame_off;
        payload.len    = chunk;
        memcpy(payload.data, buffer + sent, chunk); // memcpy(Target address, source address, number of bytes to copy)

        // Actual send length = 12 byte header + chunk data
        send_frame(U_B2A_WRITE,
                   (uint8_t*)&payload,
                   sizeof(payload.lba) + sizeof(payload.offset) + sizeof(payload.len) + chunk);
        sent += chunk;

        #ifdef DEBUG
        Serial.printf("  -> sent frame: LBA=%lu, off=%lu, len=%lu (total=%lu/%lu)\n",
                      (unsigned long)frame_lba,
                      (unsigned long)frame_off,
                      (unsigned long)chunk,
                      (unsigned long)(sent + chunk),
                      (unsigned long)bufsize);
        Serial.println("[MSC] B board WRITEDONE");
         #endif

        delay(1); // Avoid UART blocking
    }
     
    // == Wait for A board WRITEDONE confirmation ==
    uint32_t t0 = millis();
    const uint32_t TIMEOUT_MS = 3000;  // Wait up to 3 seconds for confirmation
    uint8_t type, len;
    uint8_t temp[32];
    bool done = false;

    while (millis() - t0 < TIMEOUT_MS) {
        if (!read_frame(type, temp, len, sizeof(temp))) continue;  // Modified. Error here, should be if !read_frame then continue, keep waiting until reply frame received.
            if (type == U_A2B_WRITEDONE) {

              #ifdef DEBUG
                Serial.println("[MSC] Got WRITEDONE from A board");
                #endif

                done = true;
                break;
            }
        
        delay(1);
    }

    if (!done) {
        Serial.println("[MSC] Timeout waiting for WRITEDONE!");
    }

    return bufsize; // Tell TinyUSB host write completed
  }


    //====== Callback function 2: PC reads data from USB stick ========
    static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
      #ifdef DEBUG  
      Serial.printf("\n[MSC] READ request: start_lba=%lu, offset=%lu, bufsize=%lu\n",
                    (unsigned long)lba,
                    (unsigned long)offset,
                    (unsigned long)bufsize);
      #endif

        // == Send request to A board ==
        struct {
            uint32_t lba;
            uint32_t offset;
            uint32_t len;
        } __attribute__((packed)) payload;

        payload.lba    = lba;
        payload.offset = offset;
        payload.len    = bufsize;

        send_frame(U_B2A_READ, (uint8_t*)&payload, sizeof(payload));

        // == Receive and reassemble ==
        uint8_t *p = (uint8_t*)buffer;
        size_t received = 0;
        uint32_t t0 = millis();

        while (received < bufsize && (millis() - t0 < 400) ) { // Receive timeout 4s break and wait PC request again 
          uint8_t type, len;
            uint8_t temp[255];

            if (!read_frame(type, temp, len, sizeof(temp))) {continue;} // Frame not complete, keep waiting

            if (len < 12) {continue; } 

            // === Parse frame header ===
            uint32_t frame_lba = (uint32_t)temp[0]  | ((uint32_t)temp[1] << 8) |
                                ((uint32_t)temp[2]  << 16) | ((uint32_t)temp[3] << 24);
            uint32_t frame_off = (uint32_t)temp[4]  | ((uint32_t)temp[5] << 8) |
                                ((uint32_t)temp[6]  << 16) | ((uint32_t)temp[7] << 24);
            uint32_t data_len  = (uint32_t)temp[8]  | ((uint32_t)temp[9] << 8) |
                                ((uint32_t)temp[10] << 16) | ((uint32_t)temp[11] << 24);

            // Calculate write position
          size_t buf_off = (frame_lba - lba) * 512 + frame_off;
          if (buf_off + data_len > bufsize) {continue; Serial.println("[MSC] overflow");}
          
          // === Distinguish different frame types ===
          if (type == U_A2B_READCONTENT) {  // Frame type with actual data
            // Must contain data segment
              if (len < 12 + data_len) {continue;}
              
              memcpy(p + buf_off, &temp[12], data_len); // Copy data_len bytes starting from 12th byte of this frame to buf_off position of buffer
              received += data_len;
          }

          else if (type == U_EMPTY) { // Frame type with all 0 (no data)
              
              memset(p + buf_off, 0, data_len); // set all 0
              received += data_len;

          }
          
          else continue;// ignore wrong frame type
          
          if (received >= bufsize) break;
          
          // Serial.printf("[MSC] Timeout: received %u/%lu bytes\n",
          //   (unsigned)received, (unsigned long)bufsize); //only print when timeout
        }          

        // Print all current buffer data, 16 bytes per line, extra line break every 16 lines
        // Serial.println("[MSC] buffer data:");
        // for (uint32_t i = 0; i < bufsize; i++) {
        //   Serial.printf("%02X ", p[i]);
        //   if ((i+1) % 16 == 0) Serial.println();
        //   if ((i+1) % 256 == 0) Serial.println(); // Extra line break every 16 lines
        // }
        // if (bufsize % 16 != 0) Serial.println();
      
    return bufsize;
    }


  static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
    Serial.printf("MSC START/STOP: power: %u, start: %u, eject: %u\n", power_condition, start, load_eject);
    
    struct {
        uint8_t power;
        uint8_t start;
        uint8_t eject;
    } __attribute__((packed)) payload;

    payload.power = power_condition;
    payload.start = start;
    payload.eject = load_eject;

    send_frame(U_B2A_STARTSTOP, (uint8_t*)&payload, sizeof(payload));

    return true;
  }

  static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == ARDUINO_USB_EVENTS) {
      arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
      switch (event_id) {
        case ARDUINO_USB_STARTED_EVENT: Serial.println("USB PLUGGED"); break;
        case ARDUINO_USB_STOPPED_EVENT: Serial.println("USB UNPLUGGED"); break;
        case ARDUINO_USB_SUSPEND_EVENT: Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en); break;
        case ARDUINO_USB_RESUME_EVENT:  Serial.println("USB RESUMED"); break;

        default: break;
      }
    }
  }

// ====== Check if USB stick is removed =====
  bool check_reinit_needed() {
  uint8_t type;
  uint8_t payload[32];
  uint8_t len;

  // Try to read a frame (non-blocking)
  if (read_frame(type, payload, len, sizeof(payload))) {
    // Check if it's a "reinitialize needed" message from A board
    if (type == U_A2B_REMOVED || type == CMD_REINIT || type == MSG_MOUSE || type == MSG_KEYBOARD) {
      Serial.println("[usb] Reinit command received from A board");
      return true;
    }
  }
  return false;
}

// ====== Remove USB stick and prepare to re-enter initialization =====
void Close_Ustick(){     
  MSC.end();
  g_usb_ready = false;
  g_Mode = NONE; // Let next loop reinitialize
}



// ================= Composite HID initialization (keyboard + mouse + USB stick) =================
void device_usb_init() {
  Serial.println("Waiting for initialization");
  // Declaration
  uint8_t type = 0, len = 0;
  uint8_t payload[32];
  const uint32_t TIMEOUT_MS = 100000; // 100 seconds
  uint32_t t0 = millis();
  
  // 1) Wait for type info
      Serial.println("Waiting frame");
    
    // 2) Before receive timeout, keep trying to read type
      while ((millis() - t0) < TIMEOUT_MS) {  // Before timeout, keep trying to read
        if (read_frame(type, payload, len, sizeof(payload))) {
          Serial.println("Got first frame");
          break; // Received frame and exit
        }
        delay(1);
      }
    
      Serial.println("Start frame choosing, current");
    // 3) Once break out (received type or timeout)
      // 3-1) Timeout set as HID device
      if (type == 0) {
      Serial.println("[usb] Timeout 100s, fallback to HID");
      USB.begin();
      g_hid.begin();
      g_mouse.begin();
      g_keyboard.begin();
      g_usb_ready = true;
      
      g_Mode = HID;
      delay(50);
      return;
      }
      
      // while(read_frame(type, payload, len, sizeof(payload))){
      // 3-2) If keyboard/mouse, mount to same HID composite device
      if ((type == MSG_MOUSE && len >= 4) or (type == MSG_KEYBOARD && len == 8)){
        USB.begin();
        g_hid.begin();
        g_mouse.begin();
        g_keyboard.begin();
        g_usb_ready = true;
        
        Serial.printf("mouse/keyboard connected");

        g_Mode = HID;
        delay(50); // Optional: give time for host to complete enumeration
        return;
      } 
      
      // 3-3) If USB disk, initialize as USB disk 
      else if (type == U_A2B_INIT && len == 8){ // Length to be adjusted by boyo
        uint32_t blk_sz  = 512;
        uint32_t blk_cnt = 2048; // Two placeholder default values 1MB, use placeholder if received less than 8B
        blk_sz  = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                  ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24); // Assemble first 4 bytes (low to high)
        blk_cnt = (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
                  ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24); // Assemble last 4 bytes (low to high)
        
        // Set MSC description info
        MSC.vendorID("ESP32");              // Max 8 chars
        MSC.productID("UDISK");        // Max 16 chars
        MSC.productRevision("1.0");         // Max 4 chars

        // Configure callbacks (at least required, otherwise compilation fails)
        MSC.onStartStop(onStartStop);
        MSC.onRead(onRead);
        MSC.onWrite(onWrite);

        MSC.mediaPresent(true);
        MSC.isWritable(true); // <-- Make USB stick read-only

        // Configure MSC first, then start USB
        MSC.begin(blk_cnt, blk_sz);
        USB.begin();
        g_usb_ready = true;

        g_Mode = USTICK;
        Serial.printf("[usb] init as MSC: %lu blocks, %lu bytes/block\n",
                      (unsigned long)blk_cnt, (unsigned long)blk_sz);
        delay(50);
        return;
      } 
  } 
// }


// ================= Main loop 1: Mouse+Keyboard: Read frames from UART → Dispatch to HID =================
void device_mouse_and_keyboard() {
  if (!g_usb_ready) { delay(1); return; }

  uint8_t type = 0, len = 0;
  uint8_t payload[8]; // Enough for 3B mouse or 8B keyboard
  

  // Try to consume all accumulated frames from serial port
  while (read_frame(type, payload, len, sizeof(payload))) {
    
    // --- Mouse: type = 0x01, payload = [buttons, dx, dy, wheel] (4B) ---
      if (type == MSG_MOUSE && len >= 4) {     // Ensure MSG_MOUSE == 0x01
      uint8_t btn = payload[0];              // bit0 L, bit1 R, bit2 M
      int8_t  dx  = (int8_t)payload[1];      // Already signed displacement
      int8_t  dy  = (int8_t)payload[2];
      int8_t  wheel = (int8_t)payload[3];    // Vertical wheel (send 0 if none)

      g_mouse.buttons(btn);
      g_mouse.move(dx, dy, wheel, 0 /*hWheel*/);
      Serial.printf("mouse btn=%02X dx=%d dy=%d wheel=%d\n", btn, dx, dy, wheel); // Debug use, print wheel value
    }

    // --- Keyboard: payload = [mod, reserved, key1..key6] (Boot 6KRO) ---
    else if (type == MSG_KEYBOARD && len == 8) {
      const uint8_t mod  = payload[0];
      const uint8_t *keys = &payload[2];   // 6 HID usage IDs

      // Assemble 8-byte keyboard report
      uint8_t report[8];
      report[0] = mod;
      report[1] = 0;
      memcpy(&report[2], keys, 6);

      // Send with "keyboard Report-ID" instead of 0(Boot). Report ID: Mouse=2, Keyboard=1
      const uint8_t REPORT_ID_KEYBOARD = 1;

      // Send keyboard info to host
      tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(report));
      //Serial.printf("kbd mod=%02X keys=%02X %02X %02X %02X %02X %02X\n", mod, keys[0],keys[1],keys[2],keys[3],keys[4],keys[5]);
    }

    // --- Exit loop when USB disk insertion received ---
    else if (type ==  U_A2B_INIT && len == 8|| type == CMD_REINIT){
       Serial.println("[HID] Received INIT frame, switching to MSC mode");

    // 1. Optional: Close HID reports 
    g_usb_ready = false;
    delay(50);

    // === 2. Parse parameters ===
    uint32_t blk_sz  = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                       ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
    uint32_t blk_cnt = (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
                       ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);

    // === 3. Initialize USB disk ===
    MSC.vendorID("ESP32");
    MSC.productID("UDISK");
    MSC.productRevision("1.0");

    MSC.onStartStop(onStartStop);
    MSC.onRead(onRead);
    MSC.onWrite(onWrite);

    MSC.mediaPresent(true);
    MSC.isWritable(true);

    MSC.begin(blk_cnt, blk_sz);
    USB.begin();   // Just begin again

    g_Mode = USTICK;
    g_usb_ready = true;

    Serial.printf("[usb] Switched to MSC: %lu blocks, %lu bytes/block\n",
                  (unsigned long)blk_cnt, (unsigned long)blk_sz);
    delay(50);
    return;
    }  

  }
  // Yield time slice, avoid watchdog
  delay(1);
}
