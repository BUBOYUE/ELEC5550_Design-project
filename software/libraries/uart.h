/*
 * Project: ELEC5550 Laser USB Passthrough (ESP32-S3)
 * File:    uart.h
 *
 * Author:  Kunze Chen
 * Institution: University of Western Australia (UWA)
 * Course:  ELEC5550 Design Project – Group 13
 *
 * Summary:
 *   UART communication protocol header for inter-board frame transmission.
 *   Implements a framed protocol with CRC16 error detection and optional
 *   FEC (Forward Error Correction) for reliable data exchange between
 *   Host ESP32-S3 and Device ESP32-S3 boards over UART/optical link.
 *
 * Protocol Structure:
 *   [STX(1)] [Type(1)] [Length(1)] [Payload(0-255)] [CRC16(2)]
 *   - STX: Start byte (0xAA)
 *   - Type: Message type (see MsgType enum)
 *   - Length: Payload length in bytes
 *   - Payload: Message-specific data
 *   - CRC16: CCITT-FALSE checksum for error detection
 *
 * Features:
 *   - Bidirectional frame-based communication
 *   - CRC16 error detection and correction
 *   - Optional FEC for enhanced reliability
 *   - Support for HID and MSC message types
 *   - Backward compatibility with legacy interfaces
 *
 * Hardware Configuration:
 *   - Default baud rate: 460800 bps
 *   - Default RX pin: GPIO 16
 *   - Default TX pin: GPIO 18
 *   - UART2 interface used for inter-board communication
 *
 * Build Notes:
 *   - Compatible with Arduino-ESP32 framework
 *   - Requires ESP32-S3 with multiple UART support
 *   - Can be adapted for optical communication links
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

// ====== Default hardware parameters (override if externally defined) ======
#ifndef BAUD
#define BAUD 460800           ///< Default UART baud rate for inter-board communication
#endif
#ifndef PIN_RX
#define PIN_RX 16            ///< Default GPIO pin for UART receive
#endif
#ifndef PIN_TX
#define PIN_TX 18            ///< Default GPIO pin for UART transmit
#endif

// ===== Frame header constants =====
static const uint8_t STX = 0xAA;  ///< Start of frame delimiter byte

// ===== Message types (extensible protocol) =====
/**
 * @brief Message type enumeration for frame-based protocol
 * 
 * Defines all supported message types for communication between
 * Host and Device ESP32-S3 boards. Each type has specific payload
 * format and handling requirements.
 */
enum MsgType : uint8_t {
  MSG_MOUSE    = 0x01, // payload: 3B -> dx, dy, btn
  MSG_KEYBOARD = 0x02, // payload: 8B -> modifier, reserved, key1..key6
  U_A2B_INIT  = 0x03, // U stick initilize info (block size, block count)
  U_B2A_WRITE, // PC write requirement
  U_A2B_WRITEDONE, // U stick send written done to PC
  U_B2A_READ, // PC Read requirement
  U_A2B_READCONTENT, // U stick send required content to PC
  U_B2A_STARTSTOP, // PC require U stick plug in/out
  U_A2B_REMOVED, // U stick Remove
  U_EMPTY, // All zero
  MSG_FEC_DATA, // FEC 
  CMD_REINIT  = 0xff, // Reinitial command
};

// ===== Public API Functions =====

/**
 * @brief Initialize UART2 interface for inter-board communication
 * 
 * Configures UART2 with default or predefined parameters for reliable
 * frame-based communication between Host and Device boards.
 */
void uart2_init();

/**
 * @brief Calculate CRC16-CCITT-FALSE checksum
 * 
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return 16-bit CRC checksum
 * 
 * Computes CRC16 checksum using CCITT-FALSE polynomial for frame
 * integrity verification. Used by both inner and outer protocol layers.
 */
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);

/**
 * @brief Send framed message with optional FEC
 * 
 * @param type Message type from MsgType enum
 * @param payload Pointer to message payload data
 * @param len Length of payload in bytes
 * @return true if frame sent successfully, false on error
 * 
 * Primary send function with integrated FEC support based on message type.
 * Automatically applies error correction for critical message types.
 */
bool send_frame(uint8_t type, const uint8_t* payload, uint8_t len);

/**
 * @brief Receive framed message with optional FEC
 * 
 * @param type Reference to store received message type
 * @param payload Buffer to store received payload
 * @param len Reference to store received payload length
 * @param max_len Maximum buffer size for payload
 * @return true if valid frame received, false if no frame or error
 * 
 * Primary receive function with integrated FEC support. Automatically
 * handles error correction and frame validation.
 */
bool read_frame(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// —— Internal functions (raw frames without FEC) - not recommended for direct use ——

/**
 * @brief Send raw frame without FEC encoding
 * 
 * @param type Message type
 * @param payload Payload data
 * @param len Payload length
 * @return true on success, false on error
 * 
 * Internal function for raw frame transmission. Used by FEC layer
 * and legacy compatibility functions.
 */
bool send_frame_nofec(uint8_t type, const uint8_t* payload, uint8_t len);

/**
 * @brief Receive raw frame without FEC decoding
 * 
 * @param type Reference to store message type
 * @param payload Buffer for payload
 * @param len Reference to store payload length  
 * @param max_len Maximum buffer size
 * @return true on success, false on error
 * 
 * Internal function for raw frame reception. Used by FEC layer
 * and legacy compatibility functions.
 */
bool read_frame_nofec(uint8_t& type, uint8_t* payload, uint8_t& len, size_t max_len);

// ===== Convenience wrapper functions =====

/**
 * @brief Send HID mouse report
 * 
 * @param dx X-axis movement (signed)
 * @param dy Y-axis movement (signed) 
 * @param btn Button state bitmask (bit0=L, bit1=R, bit2=M)
 * @return true on success, false on error
 * 
 * Convenience function to send mouse movement and button data
 * using the standard 3-byte HID mouse report format.
 */
bool sendMouseReport(uint8_t dx, uint8_t dy, uint8_t btn);

/**
 * @brief Send HID keyboard report
 * 
 * @param modifier Modifier key bitmask (Ctrl, Shift, Alt, GUI)
 * @param keys Array of 6 HID key codes (6KRO support)
 * @return true on success, false on error
 * 
 * Convenience function to send keyboard state using standard
 * 8-byte HID boot keyboard report format.
 */
bool sendKeyboardReport(uint8_t modifier, const uint8_t keys[6]);

/**
 * @brief Read HID mouse report
 * 
 * @param dx Reference to store X movement
 * @param dy Reference to store Y movement
 * @param btn Reference to store button state
 * @return true if mouse data received, false otherwise
 * 
 * Convenience function to receive and parse mouse report data.
 */
bool readMouse(uint8_t& dx, uint8_t& dy, uint8_t& btn);

/**
 * @brief Read HID keyboard report
 * 
 * @param modifier Reference to store modifier state
 * @param keys_out Array to store 6 key codes
 * @return true if keyboard data received, false otherwise
 * 
 * Convenience function to receive and parse keyboard report data.
 */
bool readKeyboard(uint8_t& modifier, uint8_t keys_out[6]);

// ===== Legacy compatibility functions (mouse 3-byte only) =====

/**
 * @brief Legacy mouse send function
 * 
 * @param dx X movement
 * @param dy Y movement  
 * @param btn Button state
 * 
 * @deprecated Use sendMouseReport() instead
 * Legacy function maintained for backward compatibility with
 * existing codebase. Sends 3-byte mouse data only.
 */
void sendFrame(uint8_t dx, uint8_t dy, uint8_t btn);

/**
 * @brief Legacy mouse read function
 * 
 * @param dx Reference to store X movement
 * @param dy Reference to store Y movement
 * @param btn Reference to store button state
 * @return true if mouse data received, false otherwise
 * 
 * @deprecated Use readMouse() instead
 * Legacy function maintained for backward compatibility.
 * Only handles 3-byte mouse data reception.
 */
bool readFrame(uint8_t& dx, uint8_t& dy, uint8_t& btn);

#endif
