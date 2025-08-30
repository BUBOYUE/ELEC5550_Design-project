#include <Arduino.h>
#include "uart.h"   // 提供：uart2_init(), send_frame(uint8_t type, const uint8_t* payload, uint8_t len)
#include "usb.h"    // 可选：若后续要把帧转给 USB HID 设备

// 若你的类型常量在 uart.h 中已定义，可删掉下面两行
#ifndef MSG_MOUSE
#define MSG_MOUSE    0x01
#endif
#ifndef MSG_KEYBOARD
#define MSG_KEYBOARD 0x02
#endif

// 电脑端通过 UART 发送“原始类型 + 原始负载”到本板：
//   - 鼠标：  TYPE = 0x01, 接着 3 字节 [dx, dy, btn]
//   - 键盘：  TYPE = 0x02, 接着 8 字节 [modifier, key1..key6, reserved]
// 本板把这段原始负载用 send_frame(type, payload, len) 统一打包（加 STX/CRC）再发送到下游。

void setup() {
  Serial.begin(115200);      // 调试口到电脑
  delay(200);
  uart2_init();              // 串口2：与对端板/下游链路通信
  Serial.println("ESP32-A: PC->UART(raw type+payload) -> send_frame()");
}

void loop() {
  enum RxState : uint8_t { WAIT_TYPE = 0, WAIT_PAYLOAD };
  static RxState  state      = WAIT_TYPE;
  static uint8_t  in_type    = 0;
  static uint8_t  target_len = 0;
  static uint8_t  idx        = 0;
  static uint8_t  buf[16]; // 足够容纳键盘8字节

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (state == WAIT_TYPE) {
      in_type = b;
      switch (in_type) {
        case MSG_MOUSE:    target_len = 3; break;  // [dx, dy, btn]
        case MSG_KEYBOARD: target_len = 8; break;  // [modifier, key1..key6, reserved]
        default:
          Serial.printf("[WARN] Unknown TYPE 0x%02X, drop\n", in_type);
          continue; // 仍停留在 WAIT_TYPE
      }
      idx   = 0;
      state = WAIT_PAYLOAD;
    } else { // WAIT_PAYLOAD
      if (idx < sizeof(buf)) buf[idx++] = b;

      if (idx >= target_len) {
        // 收齐一包原始负载 -> 用 send_frame() 统一打包转发
        bool ok = send_frame(in_type, buf, target_len);

        if (in_type == MSG_MOUSE) {
          // buf[0]=dx, buf[1]=dy, buf[2]=btn
          Serial.printf("[FWD mouse] dx=%d dy=%d btn=0x%02X (%s)\n",
                        (int8_t)buf[0], (int8_t)buf[1], buf[2], ok ? "ok" : "fail");
        } else if (in_type == MSG_KEYBOARD) {
          // buf[0]=modifier, buf[1..6]=keys, buf[7]=reserved
          Serial.printf("[FWD kbd] mod=0x%02X keys={%02X %02X %02X %02X %02X %02X} rsv=%02X (%s)\n",
                        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                        ok ? "ok" : "fail");
        }

        // 一帧完成，等待下一帧
        state = WAIT_TYPE;
      }
    }
  }

  // 让出一点 CPU，避免忙等
  delay(1);
}


#ifdef MOUSE_MODE
void setup() {
  //pinMode(PIN_POWER_ON, OUTPUT);
  //digitalWrite(PIN_POWER_ON, HIGH);
  Serial.begin(115200);//电脑调试串口
  delay(200);
  uart2_init();
  Serial.println("Sender ready");
}


void loop(){

    static uint32_t t0 = 0;//t0 用来记“上一次发送的时间戳”
    // if (millis() - t0 >= 1000) { // 每100ms发送一帧；millis返回开机到现在的毫秒数
    // t0 = millis();//更新“上次发送”的时间戳，为下一个 100ms 周期做准备。
    // uint8_t dx = random(0, 5);
    // uint8_t dy = random(0, 5);
    // uint8_t btn = 0x01; // 演示
    // sendFrame(dx, dy, btn);// 打包 -> 贴CRC -> 通过 Serial2 发出去
    // Serial.printf("TX: dx=%u dy=%u btn=%u\n", dx, dy, btn);// 打印到电脑调试口
    // }

    static uint8_t rx_buf[3];
    static uint8_t rx_index = 0;

    while (Serial2.available()) {
      rx_buf[rx_index++] = Serial2.read();
      if (rx_index == 3) {
        uint8_t dx = rx_buf[0];
        uint8_t dy = rx_buf[1];
        uint8_t btn = rx_buf[2];
        sendFrame(dx, dy, btn);  // 打包发送
        Serial.printf("RX->TX: dx=%u dy=%u btn=%u\n", dx, dy, btn);
        rx_index = 0;  // 重置索引，准备接收下一帧
      }
}

#endif
