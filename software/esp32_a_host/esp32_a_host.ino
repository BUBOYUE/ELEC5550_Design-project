#include <Arduino.h>
#include "uart.h"  // 需要能看到 sendFrame(...) 的声明；若你的 uart.h 只有函数原型，这里就OK
#include "usb.h"


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
    if (millis() - t0 >= 1000) { // 每100ms发送一帧；millis返回开机到现在的毫秒数
    t0 = millis();//更新“上次发送”的时间戳，为下一个 100ms 周期做准备。
    uint8_t dx = random(0, 5);
    uint8_t dy = random(0, 5);
    uint8_t btn = 0x01; // 演示
    sendFrame(dx, dy, btn);// 打包 -> 贴CRC -> 通过 Serial2 发出去
    Serial.printf("TX: dx=%u dy=%u btn=%u\n", dx, dy, btn);// 打印到电脑调试口
    }
}