## Version Information

- **Version:** 3.0 (Final Release)
- **Date:** 2025-10-19
- **Editor:** Boyue BU
- **Description:**
  - Completed the final version of the project (software, hardware, and documentation updated).
  - Added compatibility note for **ESP32-S3 DevKitC V1**, allowing firmware testing without the custom PCB.
  - Improved **Host/Device** dual communication protocol and FEC verification mechanism.
  - Updated **README.md** with library linking instructions and board compatibility notes.
  - Added **BSD 3-Clause License** for attribution-based redistribution.
  - Expanded **References** section with official ESP-IDF and TinyUSB sources.
  - Adjusted folder structure for correct GitHub rendering.
  - Enhanced TinyUSB HID/MSC framework and debugging interfaces.


# ELEC5550 Design Project – Laser-Based USB Communication System

This repository contains the complete design files for the **ELEC5550 Design Project** at the **University of Western Australia (UWA)**.  
The goal of this project is to develop a **dual ESP32-S3 optical communication bridge** that enables transparent data transfer between a **PC** and **USB devices** (keyboard, mouse, or mass storage) through a **laser-based optical link**.

---

## 📁 Project Structure

```
ELEC5550_Design-project/
│
├── docs/                         # datasheets,figures
│
├── hardware/                     # Hardware design files
│   ├── schematics/               # Circuit schematics (KiCad / Altium)
│   ├── pcb/                      # PCB layout and Gerber files
│   └── sim/                      # Simulation on LTspice
│
├── software/                     # Firmware source code for ESP32-S3 boards
│   ├── esp32_a_host/             # Host-side firmware (USB Host + UART TX)
│   ├── esp32_b_device/           # Device-side firmware (UART RX + USB Device)
│   ├── esp32s3_usb_passthrough/  # Combined HID + MSC passthrough final demo
│   └── libraries/                # Local Arduino libraries (CRC, UART, FEC, etc.)
│
├── tools/                        # Utility scripts or helper tools
│
└── README.md 
```

---

## 🧩 Hardware Compatibility

This project is designed for the **ESP32-S3 WROOM N16R8** module used on the custom PCB.  
If you do not have access to the fabricated PCB, the firmware can still be tested using an **ESP32-S3 DevKitC V1** board.  
Simply connect the UART interface between two boards according to the schematic, and the program will run with equivalent functionality.

---

## ⚙️ Setup Guide

### 1. Clone the Repository
```bash
git clone https://github.com/BUBOYUE/ELEC5550_Design-project.git
```

### 2. Link the Local Libraries
To ensure the Arduino IDE can locate the custom libraries used in this project, create symbolic links from the project’s `libraries` folder to your Arduino library path.

**Mac/Linux:**
```bash
ln -s <path_to_project>/software/libraries ~/Documents/Arduino/libraries/ELEC5550_LIBS
```

**Windows (PowerShell):**
```powershell
New-Item -ItemType SymbolicLink -Path "$env:USERPROFILE\Documents\Arduino\libraries\ELEC5550_LIBS" -Target "<path_to_project>\software\libraries"
```

### 3. Open and Flash
1. Open `esp32_a_host.ino` or `esp32_b_device.ino` using Arduino IDE.  
2. Select **Board:** `ESP32S3 Dev Module`.  
3. Set **Upload Speed:** `460800`.  
4. Enable **USB CDC On Boot** if available.  
5. Connect the board and upload.

---

## 💡 Project Overview

- **Architecture:** Dual ESP32-S3 optical bridge  
- **Communication Link:** UART or laser-based optical channel  
- **Supported Protocols:** HID (mouse/keyboard) + MSC (USB mass storage)  
- **Error Handling:** CRC-16 + Reed–Solomon FEC  
- **Operating Voltage:** 5V / 1A (from PC or power bank)

The host board (ESP32-A) acts as a USB Host, while the device board (ESP32-B) emulates a USB peripheral, enabling transparent USB data transmission over light.

---

## 🧠 Contributors
- **Boyue BU** – Software (Host-side, USB HID/MSC RTOS structure)
- **Elyney OU** – Software (Device-side, TinyUSB HID/MSC logic)
- **Kunze CHEN** – Software (UART Communication Protocol, Error Correction)
- **Zhanjun XU** – Hardware (Laser Driver Design & PCB design)
- **Zhe WANG** – Hardware (Receiver Design & PCB design)
- **Bowen LIU** – Hardware (Receiver Design & PCB design)

---

## 🧩 References
- [ESP-IDF TinyUSB Examples](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb)
- [Arduino-ESP32 USBMSC Example](https://github.com/espressif/arduino-esp32/blob/master/libraries/USB/examples/USBMSC/USBMSC.ino)
- [TinyUSB Library](https://github.com/hathach/tinyusb)
- [Reed–Solomon C++ Implementation (mersinvald)](https://github.com/mersinvald/Reed-Solomon/blob/master/include/rs.hpp)
- [CRC Algorithms Reference (Boost.CRC library, C++)](https://www.boost.org/doc/libs/release/libs/crc/crc.html)
- [ESP-IDF USB Host HID Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/hid)
- [ESP-IDF USB Host MSC Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc)
---


## 📜 License
This project is licensed under the [BSD 3-Clause License](./LICENSE).  
© 2025 ELEC5550 Optical Communication Team, University of Western Australia.  
Redistribution is permitted with acknowledgment of the original authors.