# ELEC5550 Design Project – Laser Communication System

本项目是 ELEC5550 课程设计项目，目标是实现基于 **激光通信** 的 **双 ESP32-S3 MUC**数据传输系统，支持 **USB 键盘/鼠标/存储设备** 数据通过 UART/激光链路传输到另一台计算机。

---

## **项目结构**

ELEC5550_Design-project/
│
├── docs/                   # 文档相关
│
├── hardware/               # 硬件设计
│   ├── schematics/         # 电路原理图（KiCAD/Altium）
│   ├── pcb/                # PCB Layout
│   └── bom/                # 元器件清单（BOM）
│
├── software/               # 软件（嵌入式代码）
│   ├── esp32_a_host.ino    # Host 板（USB Host + UART 发送）
│   ├── esp32_b_device.ino  # Device 板（UART 接收 + USB Device 输出）
│   └── libraries/          # 项目内局部库
│       ├── common/         # 公共模块（UART、CRC、帧封装）
│       ├── host_lib/       # Host 模块（USB Host 逻辑）
│       └── device_lib/     # Device 模块（USB HID 逻辑）
│
├── tools/                  # 辅助脚本/工具
│
└── README.md               # 本文件

## **版本信息**
- **Version:** 1.0 (Initial Version)
- **Date:** 2025-08-03
- **Editor** BOYUE BU
- **description**  
  - 初始项目结构建立（docs/hardware/software/tools）  
  - 创建 Host/Device `.ino` 框架  
  - 建立 `common/host_lib/device_lib` 三个本地库目录  
  - 预留文档和硬件目录结构

---

## **使用方法**
1. 克隆仓库：
   ```bash
   git clone https://github.com/<yourusername>/ELEC5550_Design-project.git