//Ustick Reference from TinyUSB examples:https://github.com/espressif/arduino-esp32/blob/master/libraries/USB/examples/USBMSC/USBMSC.ino
#include "device.h"
#include "uart.h"
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
#include "USBMSC.h"
#include "tusb.h"   // 直接用 TinyUSB 的底层 API 发送键盘报告

#define DEBUG 0  //debug mode


// ===== 全局对象：HID 总线 + 设备 =====
static USBHID              g_hid;           // HID 总线
static USBHIDRelativeMouse g_mouse;         // 相对坐标鼠标（默认构造）
static USBHIDKeyboard      g_keyboard;      // 键盘（默认构造）
static USBMSC              MSC;             // U盘设备对象
bool   g_usb_ready = false;    //初始化前认为鼠标还没准备好
int    g_Mode = NONE;  // Device Mode


//==================回调函数要写在初始化之前=====================

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  #ifdef DEBUG
  Serial.printf("MSC WRITE: lba: %lu, offset: %lu, bufsize: %lu\n", lba, offset, bufsize);
    #endif
    const uint16_t MAX_PAYLOAD = 224;   // 最大单帧数据负载
    uint32_t sent = 0; // 已发送字节数

    // == 发送 B 板 WRITE 请求 ==
    while (sent < bufsize) {
        // 本帧发送的数据长度
        uint32_t chunk = bufsize - sent;
        if (chunk > MAX_PAYLOAD) chunk = MAX_PAYLOAD;

        // 计算该帧的绝对扇区位置
        uint32_t absolute  = offset + sent;
        uint32_t frame_lba = lba + (absolute / 512);   // 跨扇区时自动进位
        uint32_t frame_off = absolute % 512;

        // 组装 payload 结构
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

        // 实际发送长度 = 12 字节头 + chunk 数据
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

        delay(1); // 避免UART阻塞
    }
     
    // == 等待 A 板确认 WRITEDONE ==
    uint32_t t0 = millis();
    const uint32_t TIMEOUT_MS = 3000;  // 最多等3秒确认信息
    uint8_t type, len;
    uint8_t temp[32];
    bool done = false;

    while (millis() - t0 < TIMEOUT_MS) {
        if (!read_frame(type, temp, len, sizeof(temp))) continue;  //已修改。此处出错，应该是if !read_frame则continue，持续等到接收到回复帧为止。
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

    return bufsize; // 告诉TinyUSB主机已写完
  }


    //======回调函数2：电脑读数据从 U 盘========
    static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
      #ifdef DEBUG  
      Serial.printf("\n[MSC] READ request: start_lba=%lu, offset=%lu, bufsize=%lu\n",
                    (unsigned long)lba,
                    (unsigned long)offset,
                    (unsigned long)bufsize);
      #endif

        //==发请求给 A 板==
        struct {
            uint32_t lba;
            uint32_t offset;
            uint32_t len;
        } __attribute__((packed)) payload;

        payload.lba    = lba;
        payload.offset = offset;
        payload.len    = bufsize;

        send_frame(U_B2A_READ, (uint8_t*)&payload, sizeof(payload));

        //==接收并拼接==
        uint8_t *p = (uint8_t*)buffer;
        size_t received = 0;
        uint32_t t0 = millis();

        while (received < bufsize && (millis() - t0 < 400) ) { //receive overtime 4s break and wait PC request again 
          uint8_t type, len;
            uint8_t temp[255];

            if (!read_frame(type, temp, len, sizeof(temp))) {continue;} // 没收完帧，继续等

            if (len < 12) {continue; } 

            // === 解析帧头 ===
            uint32_t frame_lba = (uint32_t)temp[0]  | ((uint32_t)temp[1] << 8) |
                                ((uint32_t)temp[2]  << 16) | ((uint32_t)temp[3] << 24);
            uint32_t frame_off = (uint32_t)temp[4]  | ((uint32_t)temp[5] << 8) |
                                ((uint32_t)temp[6]  << 16) | ((uint32_t)temp[7] << 24);
            uint32_t data_len  = (uint32_t)temp[8]  | ((uint32_t)temp[9] << 8) |
                                ((uint32_t)temp[10] << 16) | ((uint32_t)temp[11] << 24);

            // 计算写入位置
          size_t buf_off = (frame_lba - lba) * 512 + frame_off;
          if (buf_off + data_len > bufsize) {continue; Serial.println("[MSC] overflow");}
          
          // === 区分不同帧类型 ===
          if (type == U_A2B_READCONTENT) {  //Framne type with actual data
            // 必须包含 data 段
              if (len < 12 + data_len) {continue;}
              
              memcpy(p + buf_off, &temp[12], data_len); //把该帧的第12字节开始的data_len字节数据，拷贝到buffer的buf_off位置
              received += data_len;
          }

          else if (type == U_EMPTY) { //Frame type with all 0 (no data)
              
              memset(p + buf_off, 0, data_len); //set all 0
              received += data_len;

          }
          
          else continue;// ignore wrong frame type
          
          if (received >= bufsize) break;
          
          // Serial.printf("[MSC] Timeout: received %u/%lu bytes\n",
          //   (unsigned)received, (unsigned long)bufsize); //only print when timeout
        }          

        // ⑦ 打印buffer当前所有数据，每16字节换行，每16行多换一次行
        // Serial.println("[MSC] buffer data:");
        // for (uint32_t i = 0; i < bufsize; i++) {
        //   Serial.printf("%02X ", p[i]);
        //   if ((i+1) % 16 == 0) Serial.println();
        //   if ((i+1) % 256 == 0) Serial.println(); // 每16行多换一次行
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

//======检查U盘是否拔出=====
  bool check_reinit_needed() {
  uint8_t type;
  uint8_t payload[32];
  uint8_t len;

  // 尝试读取一帧（非阻塞）
  if (read_frame(type, payload, len, sizeof(payload))) {
    // 判断是不是 A 板发来的“需要重初始化”的消息
    if (type == U_A2B_REMOVED || type == CMD_REINIT || type == MSG_MOUSE || type == MSG_KEYBOARD) {
      Serial.println("[usb] Reinit command received from A board");
      return true;
    }
  }
  return false;
}

//======拔出U盘准备重新进入初始化=====
void Close_Ustick(){     
  MSC.end();
  g_usb_ready = false;
  g_Mode = NONE; // 让下一轮 loop 重新初始化
}



// ================= 复合 HID 初始化（键盘 + 鼠标 + U盘）=================
void device_usb_init() {
  Serial.println("Waiting for initialization");
  //declaration
  uint8_t type = 0, len = 0;
  uint8_t payload[32];
  const uint32_t TIMEOUT_MS = 100000; // 100 秒
  uint32_t t0 = millis();
  
  // 1)wait for type info
      Serial.println("Waiting frame");
    
    // 2) before receive overtime, keep trying read type
      while ((millis() - t0) < TIMEOUT_MS) {  //before overtime, keep trying read
        if (read_frame(type, payload, len, sizeof(payload))) {
          Serial.println("Got first frame");
          break; // received frame and out
        }
        delay(1);
      }
    
      Serial.println("Start frame choosing, current");
    // 3)Once break out (received type or overtime)
      // 3-1)overtime set as HID device
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
      // 3-2) 如果是键盘/鼠标, 挂载到同一个 HID 复合设备上
      if ((type == MSG_MOUSE && len >= 4) or (type == MSG_KEYBOARD && len == 8)){
        USB.begin();
        g_hid.begin();
        g_mouse.begin();
        g_keyboard.begin();
        g_usb_ready = true;
        
        Serial.printf("mouse/keyboard connected");

        g_Mode = HID;
        delay(50); // 可选：给点时间让主机完成枚举
        return;
      } 
      
      // 3-3) if u disk，initialize as u disk 
      else if (type == U_A2B_INIT && len == 8){ //长度待boyo调整
        uint32_t blk_sz  = 512;
        uint32_t blk_cnt = 2048; // 两个占位默认值 1MB，若收到不足8B则用占位
        blk_sz  = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                  ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24); // 拼装前四字节（从低到高）
        blk_cnt = (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
                  ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24); // 拼装后四字节（从低到高）
        
        // 设置 MSC 描述信息
        MSC.vendorID("ESP32");              // 最多 8 chars
        MSC.productID("UDISK");        // 最多 16 chars
        MSC.productRevision("1.0");         // 最多 4 chars

        // 配置回调（至少要有，不然编译不过）
        MSC.onStartStop(onStartStop);
        MSC.onRead(onRead);
        MSC.onWrite(onWrite);

        MSC.mediaPresent(true);
        MSC.isWritable(true); // <-- 让U盘只读

        // 先配置 MSC，再启动 USB
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


// ================= 主循环1：鼠标+键盘：从 UART 读帧 → 分发到 HID =================
void device_mouse_and_keyboard() {
  if (!g_usb_ready) { delay(1); return; }

  uint8_t type = 0, len = 0;
  uint8_t payload[8]; // 够装 3B 鼠标或 8B 键盘
  

  // 把串口里累积的帧尽量吃干净
  while (read_frame(type, payload, len, sizeof(payload))) {
    
    // --- 鼠标：type = 0x01, payload = [buttons, dx, dy, wheel] (4B)---
      if (type == MSG_MOUSE && len >= 4) {     // 确保 MSG_MOUSE == 0x01
      uint8_t btn = payload[0];              // bit0 L, bit1 R, bit2 M
      int8_t  dx  = (int8_t)payload[1];      // 已是有符号位移
      int8_t  dy  = (int8_t)payload[2];
      int8_t  wheel = (int8_t)payload[3];    // 垂直滚轮（无则发0）

      g_mouse.buttons(btn);
      g_mouse.move(dx, dy, wheel, 0 /*hWheel*/);
      Serial.printf("mouse btn=%02X dx=%d dy=%d wheel=%d\n", btn, dx, dy, wheel); //调试用，打印滚轮值
    }

    // --- 键盘：payload = [mod, reserved, key1..key6] (Boot 6KRO) ---
    else if (type == MSG_KEYBOARD && len == 8) {
      const uint8_t mod  = payload[0];
      const uint8_t *keys = &payload[2];   // 6 个 HID usage ID

      // 组装 8 字节键盘报告
      uint8_t report[8];
      report[0] = mod;
      report[1] = 0;
      memcpy(&report[2], keys, 6);

      // 用“键盘的 Report-ID”发送，而不是0(Boot).Report ID: Mouse=2,Keyboard=1
      const uint8_t REPORT_ID_KEYBOARD = 1;

      // 将键盘信息发送到主机
      tud_hid_report(REPORT_ID_KEYBOARD, report, sizeof(report));
      //Serial.printf("kbd mod=%02X keys=%02X %02X %02X %02X %02X %02X\n", mod, keys[0],keys[1],keys[2],keys[3],keys[4],keys[5]);
    }

    // --- 接收到U盘插入时跳出循环 ---
    else if (type ==  U_A2B_INIT && len == 8|| type == CMD_REINIT){
       Serial.println("[HID] Received INIT frame, switching to MSC mode");

    // 1. 可选：关闭 HID 报告 
    g_usb_ready = false;
    delay(50);

    // === 2. 解析参数 ===
    uint32_t blk_sz  = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                       ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
    uint32_t blk_cnt = (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
                       ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);

    // === 3. 初始化 U盘 ===
    MSC.vendorID("ESP32");
    MSC.productID("UDISK");
    MSC.productRevision("1.0");

    MSC.onStartStop(onStartStop);
    MSC.onRead(onRead);
    MSC.onWrite(onWrite);

    MSC.mediaPresent(true);
    MSC.isWritable(true);

    MSC.begin(blk_cnt, blk_sz);
    USB.begin();   // 直接再 begin 一次即可

    g_Mode = USTICK;
    g_usb_ready = true;

    Serial.printf("[usb] Switched to MSC: %lu blocks, %lu bytes/block\n",
                  (unsigned long)blk_cnt, (unsigned long)blk_sz);
    delay(50);
    return;
    }  

  }
  // 让出时间片，避免看门狗
  delay(1);
}
