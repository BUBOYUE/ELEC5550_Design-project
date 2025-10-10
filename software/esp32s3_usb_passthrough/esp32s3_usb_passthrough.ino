#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "usb/usb_host.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

#include "msc_host.h"
#include "msc_host_vfs.h"

#include "uart.h"   // 提供：uart2_init(), send_frame(...)
#include "usb.h"    // 可选：若后续把帧转给 USB HID 设备

#include "device.h"   // 声明了 device_usb_init / device_mouse_and_keyboard / Close_Ustick 等

static const char *TAG = "example";

#ifndef MNT_PATH
#define MNT_PATH "/usb"
#endif
#ifndef BUFFER_SIZE
#define BUFFER_SIZE (4 * 1024)
#endif
#ifndef APP_QUIT_PIN
#define APP_QUIT_PIN GPIO_NUM_0
#endif
// IMPORTANT NOTE
// MSC Class Driver is not fully support connecting devices through external Hub.
// TODO: Remove this line after MSC Class Driver will support it
static bool dev_present = false;

static uint32_t g_sector_size = 512;                 // 从设备信息更新
static msc_host_device_handle_t g_msc_dev = NULL;    // 当前 MSC 设备句柄
static const uint8_t CHUNK_DATA_MAX = 224;           // 单帧分片最大数据长度（从240改为224）

/**
 * @brief Application Queue and its messages ID
 */
static QueueHandle_t app_queue;
//bby typedef struct {
//     enum {
//         APP_QUIT,                // Signals request to exit the application
//         APP_DEVICE_CONNECTED,    // USB device connect event
//         APP_DEVICE_DISCONNECTED, // USB device disconnect event
//     } id;
//     union {
//         uint8_t new_dev_address; // Address of new USB device for APP_DEVICE_CONNECTED event if
//     } data;
// } app_message_t;

typedef enum {
    APP_QUIT = 0,             // 显式定义数值
    APP_DEVICE_CONNECTED,
    APP_DEVICE_DISCONNECTED,
    APP_COMMAND,//bby
} app_event_id_t;

// typedef struct {
//     app_event_id_t id;        // 用刚才定义的类型
//     union {
//         uint8_t new_dev_address;
//     } data;
// } app_message_t;


typedef struct {
    app_event_id_t id;
    union {
        uint8_t new_dev_address;
        struct {                   // 用于 UART 命令转发
            uint8_t type;          // 帧类型（MsgType）
            uint8_t data[256];     // 负载
            uint8_t len;           // 负载长度
        } cmd;
    } data;
} app_message_t;



// 电脑端通过 UART 发送“原始类型 + 原始负载”到本板：
//   - 鼠标：  TYPE = 0x01, 接着 3 字节 [dx, dy, btn]
//   - 键盘：  TYPE = 0x02, 接着 8 字节 [modifier, key1..key6, reserved]
// 本板把这段原始负载用 send_frame(type, payload, len) 统一打包（加 STX/CRC）再发送到下游。

/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN                GPIO_NUM_0

QueueHandle_t hid_host_event_queue;
bool user_shutdown = false;

/**
 * @brief HID Host event
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  hid_host_device_handle_t hid_device_handle;
  hid_host_driver_event_t event;
  void *arg;
} hid_host_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {"NONE", "KEYBOARD", "MOUSE"};

/**
 * @brief Key event
 */
typedef struct {
  enum key_state { KEY_STATE_PRESSED = 0x00, KEY_STATE_RELEASED = 0x01 } state;
  uint8_t modifier;
  uint8_t key_code;
} key_event_t;

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial
 * debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
    {0, 0},     /* HID_KEY_NO_PRESS        */
    {0, 0},     /* HID_KEY_ROLLOVER        */
    {0, 0},     /* HID_KEY_POST_FAIL       */
    {0, 0},     /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER */
    {0, 0},      /* HID_KEY_ESC             */
    {'\b', 0},   /* HID_KEY_DEL             */
    {0, 0},      /* HID_KEY_TAB             */
    {' ', ' '},  /* HID_KEY_SPACE           */
    {'-', '_'},  /* HID_KEY_MINUS           */
    {'=', '+'},  /* HID_KEY_EQUAL           */
    {'[', '{'},  /* HID_KEY_OPEN_BRACKET    */
    {']', '}'},  /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'},
    /* HID_KEY_SHARP           */ // HOTFIX: for NonUS Keyboards repeat
                                  // HID_KEY_BACK_SLASH
    {';', ':'},                   /* HID_KEY_COLON           */
    {'\'', '"'},                  /* HID_KEY_QUOTE           */
    {'`', '~'},                   /* HID_KEY_TILDE           */
    {',', '<'},                   /* HID_KEY_LESS            */
    {'.', '>'},                   /* HID_KEY_GREATER         */
    {'/', '?'}                    /* HID_KEY_SLASH           */
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto) {
  static hid_protocol_t prev_proto_output = HID_PROTOCOL_MAX;

  if (prev_proto_output != proto) {
    prev_proto_output = proto;
    printf("\r\n");
    if (proto == HID_PROTOCOL_MOUSE) {
      printf("Mouse\r\n");
    } else if (proto == HID_PROTOCOL_KEYBOARD) {
      printf("Keyboard\r\n");
    } else {
      printf("Generic\r\n");
    }
    fflush(stdout);
  }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application
 * (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier) {
  if (((modifier & HID_LEFT_SHIFT) == HID_LEFT_SHIFT) ||
      ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT)) {
    return true;
  }
  return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier, uint8_t key_code,
                                         unsigned char *key_char) {
  uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

  if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH)) {
    *key_char = keycode2ascii[key_code][mod];
  } else {
    // All other key pressed
    return false;
  }

  return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char) {
  if (!!key_char) {
    putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
    if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
      putchar('\n');
    }
#endif // KEYBOARD_ENTER_LF_EXTEND
    fflush(stdout);
  }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event) {
  unsigned char key_char;

  hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

  if (key_event->KEY_STATE_PRESSED == key_event->state) {
    if (hid_keyboard_get_char(key_event->modifier, key_event->key_code,
                              &key_char)) {

      hid_keyboard_print_char(key_char);
    }
  }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src, uint8_t key,
                             unsigned int length) {
  for (unsigned int i = 0; i < length; i++) {
    if (src[i] == key) {
      return true;
    }
  }
  return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data,
                                              const int length) {
  hid_keyboard_input_report_boot_t *kb_report =
      (hid_keyboard_input_report_boot_t *)data;

  if (length < sizeof(hid_keyboard_input_report_boot_t)) {
    return;
  }

  // DEBUG: one-line dump of keyboard report
  printf("[DBG][KB] size=%d hex:", length);
  for (int i = 0; i < length; ++i) printf(" %02X", data[i]);
  printf("\r\n");
  fflush(stdout);
  
  // ---- Send raw Boot keyboard snapshot via UART (type=0x02) ----
  // Payload layout (8 bytes): [0]=modifier, [1]=reserved(0), [2..7]=key[6]
  uint8_t kb_payload[8];
  kb_payload[0] = kb_report->modifier.val; // modifiers (Ctrl/Shift/Alt/GUI)
  kb_payload[1] = 0x00;                    // reserved
  memcpy(&kb_payload[2], kb_report->key, 6);
  send_frame(0x02, kb_payload, 8);

  static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
  key_event_t key_event;

  for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {

    // key has been released verification
    if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = prev_keys[i];
      key_event.modifier = 0;
      key_event.state = key_event.KEY_STATE_RELEASED;
      key_event_callback(&key_event);
    }

    // key has been pressed verification
    if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
        !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
      key_event.key_code = kb_report->key[i];
      key_event.modifier = kb_report->modifier.val;
      key_event.state = key_event.KEY_STATE_PRESSED;
      key_event_callback(&key_event);
    }
  }

  memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
// bby static void hid_host_mouse_report_callback(const uint8_t *const data,
//                                            const int length){ 
//   hid_mouse_input_report_boot_t *mouse_report =
//       (hid_mouse_input_report_boot_t *)data;

//   if (length < sizeof(hid_mouse_input_report_boot_t)) {
//     return;
//   }

//   static int x_pos = 0;
//   static int y_pos = 0;

//   // Calculate absolute position from displacement
//   x_pos += mouse_report->x_displacement;
//   y_pos += mouse_report->y_displacement;

//   hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

//   printf("X: %06d\tY: %06d\t|%c|%c|\r", x_pos, y_pos,
//          (mouse_report->buttons.button1 ? 'o' : ' '),
//          (mouse_report->buttons.button2 ? 'o' : ' '));
//   fflush(stdout);
// }

static void hid_host_mouse_report_callback(const uint8_t *const data,
                                           const int length){
  // DEBUG: dump raw mouse report as signed decimal (int8)
  printf("[DBG][MOUSE] size=%d s8:", length);
  for (int i = 0; i < length; ++i) printf(" %4d", (int)((int8_t)data[i]));
  printf("\r\n");
  fflush(stdout);
  // 自适配 Boot/Report 常见布局；见上三种格式
  const uint8_t *p = data;
  int remaining = length;
  if (remaining < 3) return;

  // 简单启发式：若第 0 字节不像按钮掩码而第 1 字节像，则认为第 0 字节是 Report ID
  bool has_report_id = false;
  if ((p[0] & 0xF8) && remaining >= 5 && (p[1] & 0xF8) == 0) {
    has_report_id = true;
    p++; remaining--;
  }

  uint8_t buttons = p[0];
  int8_t dx = (remaining >= 2) ? (int8_t)p[1] : 0;
  // ----- Robust Y: treat p[3] as signed high byte, then scale by 8 with signed rounding -----
  int16_t dy16 = 0;
  if (remaining >= 4) {
    // Little-endian 16-bit: low=p[2], high=p[3] (high is signed)
    int8_t dy_hi = (int8_t)p[3];
    uint8_t dy_lo = (remaining >= 3) ? p[2] : 0;
    dy16 = (int16_t)(((int16_t)dy_hi << 8) | (uint16_t)dy_lo);
  } else {
    // Fallback to 8-bit
    dy16 = (int16_t)((remaining >= 3) ? (int8_t)p[2] : 0);
  }
  // Signed rounding division by 8 to match desired sensitivity
  int dy_scaled = (dy16 >= 0) ? (((int)dy16 + 4) / 8) : (((int)dy16 - 4) / 8);
  if (dy_scaled > 127) dy_scaled = 127;
  if (dy_scaled < -128) dy_scaled = -128;
  int8_t dy = (int8_t)dy_scaled;
  // Debug print for combined Y values
  printf("[DBG][MOUSE] dy_lo=0x%02X dy_hi=0x%02X dy16=%d dy=%d\r\n", (remaining>=3?p[2]:0), (remaining>=4?p[3]:0), (int)dy16, (int)dy);
  fflush(stdout);
  int8_t wheel = 0;
  int8_t wheel_h = 0; // optional horizontal wheel

  if (remaining == 6) {
    // Observed format from your logs:
    // [0]=buttons, [1]=dx, [2]=dy, [3]=horiz(wheel H?), [4]=wheel V, [5]=reserved
    wheel_h = (int8_t)p[3];
    wheel   = (int8_t)p[4];
  } else if (remaining >= 4) {
    // Default common format: [buttons][dx][dy][wheel]
    wheel = (int8_t)p[3];
  }

  // Pack useful 4-byte payload: [buttons(2:中 1:右 0:左), dx, dy, wheel]
  uint8_t payload[4];
  payload[0] = buttons;
  payload[1] = (uint8_t)dx;
  payload[2] = (uint8_t)dy;
  payload[3] = (uint8_t)wheel;
  send_frame(0x01, payload, 4);


  hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);
  printf("X:%06d\tY:%06d\t", dx, dy);
  if (remaining >= 4) printf("W:%4d\t", (int)wheel);
  printf("|%c|%c|%c|%s\r \n",
         (buttons & 0x01) ? 'o' : ' ',
         (buttons & 0x02) ? 'o' : ' ',
         (buttons & 0x04) ? 'o' : ' ',
         has_report_id ? "RID" : "");
  fflush(stdout);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data,
                                             const int length) {
  hid_print_new_device_report_header(HID_PROTOCOL_NONE);
  for (int i = 0; i < length; i++) {
    printf("%02X", data[i]);
  }
  putchar('\r');
  putchar('\n');
  fflush(stdout);
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg) {
  uint8_t data[64] = {0};
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(
        hid_device_handle, data, 64, &data_length));

    //bby if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        hid_host_keyboard_report_callback(data, data_length);
      } else if (HID_PROTOCOL_MOUSE == dev_params.proto) {
        hid_host_mouse_report_callback(data, data_length);
      } else {
        hid_host_generic_report_callback(data, data_length);
      }

    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
             hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
             hid_proto_name_str[dev_params.proto]);
    break;
  default:
    ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
             hid_proto_name_str[dev_params.proto]);
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event, void *arg) {
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));
  const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_callback, .callback_arg = NULL};


  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED:
    ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
             hid_proto_name_str[dev_params.proto]);

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle,
                                                     HID_REPORT_PROTOCOL_REPORT));
                                                    //  HID_REPORT_PROTOCOL_BOOT));//BBY
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
      }
    }
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
    break;
  default:
    break;
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while
 * app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg) {
  const gpio_config_t input_pin = {
      .pin_bit_mask = BIT64(APP_QUIT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_pin));

  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive((TaskHandle_t)arg);

  while (gpio_get_level(APP_QUIT_PIN) != 0) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

    // Release devices once all clients has deregistered
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
      ESP_LOGI(TAG, "USB Event flags: NO_CLIENTS");
    }
    // All devices were removed
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB Event flags: ALL_FREE");
    }
  }
  // App Button was pressed, trigger the flag
  user_shutdown = true;
  ESP_LOGI(TAG, "USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}

/**
 * @brief HID Host main task
 *
 * Creates queue and get new event from the queue
 *
 * @param[in] pvParameters Not used
 */
void hid_host_task(void *pvParameters) {
  hid_host_event_queue_t evt_queue;
  // Create queue
  hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

  // Wait queue
  while (!user_shutdown) {
    if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50))) {
      hid_host_device_event(evt_queue.hid_device_handle, evt_queue.event,
                            evt_queue.arg);
    }
  }

  xQueueReset(hid_host_event_queue);
  vQueueDelete(hid_host_event_queue);
  vTaskDelete(NULL);
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event, void *arg) {
  const hid_host_event_queue_t evt_queue = {
      .hid_device_handle = hid_device_handle, .event = event, .arg = arg};
  xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

//-----------------MSC相关-----------------

/**
 * @brief BOOT button pressed callback
 *
 * Signal application to exit the main task
 *
 * @param[in] arg Unused
 */
static void gpio_cb(void *arg)
{
    BaseType_t xTaskWoken = pdFALSE;
    app_message_t message = {
        .id = APP_QUIT,
    };

    if (app_queue) {
        xQueueSendFromISR(app_queue, &message, &xTaskWoken);
    }

    if (xTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief MSC driver callback
 *
 * Signal device connection/disconnection to the main task
 *
 * @param[in] event MSC event
 * @param[in] arg   MSC event data
 */
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    if (event->event == MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected (usb_addr=%d)", event->device.address);
        app_message_t message = {
            .id = APP_DEVICE_CONNECTED,
            // .data.new_dev_address = event->device.address,bby
            .data = { .new_dev_address = event->device.address },
        };
        xQueueSend(app_queue, &message, portMAX_DELAY);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC device disconnected");
        app_message_t message = {
            .id = APP_DEVICE_DISCONNECTED,
        };
        xQueueSend(app_queue, &message, portMAX_DELAY);
    }
}

static void print_device_info(msc_host_device_info_t *info)
{
    const size_t megabyte = 1024 * 1024;
    uint64_t capacity = ((uint64_t)info->sector_size * info->sector_count) / megabyte;

    printf("Device info:\n");
    printf("\t Capacity: %llu MB\n", capacity);
    printf("\t Sector size: %"PRIu32"\n", info->sector_size);
    printf("\t Sector count: %"PRIu32"\n", info->sector_count);
    printf("\t PID: 0x%04X \n", info->idProduct);
    printf("\t VID: 0x%04X \n", info->idVendor);
    wprintf(L"\t iProduct: %S \n", info->iProduct);
    wprintf(L"\t iManufacturer: %S \n", info->iManufacturer);
    wprintf(L"\t iSerialNumber: %S \n", info->iSerialNumber);

    struct {
        uint32_t sector_size;
        uint32_t sector_count;
    } init_info;

    init_info.sector_size = info->sector_size;
    init_info.sector_count = info->sector_count;

    // NOTE to receiver:
    // The payload of this U_INIT frame contains two fields:
    //   uint32_t sector_size
    //   uint32_t sector_count
    // They are packed directly as raw 32-bit integers (little-endian).
    // Because send_frame transmits as bytes (uint8_t*), read_frame() on the
    // receiving side must reassemble 4 bytes into a uint32_t for each field.
    // Example (little-endian):
    //   sector_size  = payload[0] | (payload[1]<<8) | (payload[2]<<16) | (payload[3]<<24);
    //   sector_count = payload[4] | (payload[5]<<8) | (payload[6]<<16) | (payload[7]<<24);

    // 添加打印消息
    Serial.printf("[UART] INIT packet sent: sector_size=%u, sector_count=%u\n", 
                  info->sector_size, info->sector_count);

    // Send initialization info upstream
    send_frame(U_A2B_INIT, (const uint8_t *)&init_info, sizeof(init_info));
    g_sector_size = info->sector_size;
    
}

static void file_operations(void)
{
    const char *directory = "/usb/esp";
    const char *file_path = "/usb/esp/test.txt";

    // Create /usb/esp directory
    struct stat s = {0};
    bool directory_exists = stat(directory, &s) == 0;
    if (!directory_exists) {
        if (mkdir(directory, 0775) != 0) {
            ESP_LOGE(TAG, "mkdir failed with errno: %s", strerror(errno));
        }
    }

    // Create /usb/esp/test.txt file, if it doesn't exist
    if (stat(file_path, &s) != 0) {
        ESP_LOGI(TAG, "Creating file");
        FILE *f = fopen(file_path, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
        }
        fprintf(f, "Hello World!\n");
        fclose(f);
    }

    // Read back the file
    FILE *f;
    ESP_LOGI(TAG, "Reading file");
    f = fopen(file_path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file '%s': '%s'", file_path, line);
}

void speed_test(void)
{
#define TEST_FILE "/usb/esp/dummy"
#define ITERATIONS  256  // 256 * 4kb = 1MB
    int64_t test_start, test_end;

    FILE *f = fopen(TEST_FILE, "wb+");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    // Set larger buffer for this file. It results in larger and more effective USB transfers
    setvbuf(f, NULL, _IOFBF, BUFFER_SIZE);

    // Allocate application buffer used for read/write
    // uint8_t *data = malloc(BUFFER_SIZE); bby
    uint8_t *data = (uint8_t *) malloc(BUFFER_SIZE);
    assert(data);

    ESP_LOGI(TAG, "Writing to file %s", TEST_FILE);
    test_start = esp_timer_get_time();
    for (int i = 0; i < ITERATIONS; i++) {
        if (fwrite(data, BUFFER_SIZE, 1, f) == 0) {
            return;
        }
    }
    test_end = esp_timer_get_time();
    ESP_LOGI(TAG, "Write speed %1.2f MiB/s", (BUFFER_SIZE * ITERATIONS) / (float)(test_end - test_start));
    rewind(f);

    ESP_LOGI(TAG, "Reading from file %s", TEST_FILE);
    test_start = esp_timer_get_time();
    for (int i = 0; i < ITERATIONS; i++) {
        if (0 == fread(data, BUFFER_SIZE, 1, f)) {
            return;
        }
    }
    test_end = esp_timer_get_time();
    ESP_LOGI(TAG, "Read speed %1.2f MiB/s", (BUFFER_SIZE * ITERATIONS) / (float)(test_end - test_start));

    fclose(f);
    free(data);
}

/**
 * @brief USB task
 *
 * Install USB Host Library and MSC driver.
 * Handle USB Host Library events
 *
 * @param[in] args Unused
 */
static void usb_task(void *args)
{
    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));
    ESP_LOGI(TAG, "MSC driver installed");
    vTaskDelete(NULL);
}

static void send_readcontent_frame(uint32_t lba, uint32_t offset, uint32_t data_len,
                                   const uint8_t* data)
{
    // 按224字节分片发送数据（从240改为224）
    uint32_t sent = 0;
    while (sent < data_len) {
        uint32_t chunk_size = (data_len - sent > 224) ? 224 : (data_len - sent); // 240改为224
        
        // 计算当前chunk对应的实际LBA和扇区内偏移
        uint32_t current_absolute_offset = offset + sent;
        uint32_t current_lba = lba + (current_absolute_offset / g_sector_size);
        uint32_t current_offset_in_sector = current_absolute_offset % g_sector_size;
        
        uint8_t buf[4 + 4 + 4 + 224]; // lba(4) + offset(4) + data_length(4) + data(最大224字节，从240改为224)
        size_t p = 0;
        
        // current_lba (LE)
        buf[p++] = (uint8_t)(current_lba & 0xFF);
        buf[p++] = (uint8_t)((current_lba >> 8) & 0xFF);
        buf[p++] = (uint8_t)((current_lba >> 16) & 0xFF);
        buf[p++] = (uint8_t)((current_lba >> 24) & 0xFF);
        
        // current_offset_in_sector (LE)
        buf[p++] = (uint8_t)(current_offset_in_sector & 0xFF);
        buf[p++] = (uint8_t)((current_offset_in_sector >> 8) & 0xFF);
        buf[p++] = (uint8_t)((current_offset_in_sector >> 16) & 0xFF);
        buf[p++] = (uint8_t)((current_offset_in_sector >> 24) & 0xFF);
        
        // data_length (LE) - 当前帧的数据长度
        buf[p++] = (uint8_t)(chunk_size & 0xFF);
        buf[p++] = (uint8_t)((chunk_size >> 8) & 0xFF);
        buf[p++] = (uint8_t)((chunk_size >> 16) & 0xFF);
        buf[p++] = (uint8_t)((chunk_size >> 24) & 0xFF);
        
        // data
        memcpy(&buf[p], data + sent, chunk_size);
        p += chunk_size;

        // debug_bby // 添加打印消息（计算公式也要改）
        Serial.printf("[MSC] Sending chunk %u/%u: lba=%u, sector_offset=%u, data_length=%u (abs_offset=%u)\n", 
                      (sent/224)+1, (data_len+223)/224, current_lba, current_offset_in_sector, chunk_size, current_absolute_offset);

        send_frame(U_A2B_READCONTENT, buf, (uint8_t)p);
        
        sent += chunk_size;
    }
}

// 新增：发送全 0 空数据帧
static void send_empty_frame(uint32_t lba, uint32_t offset, uint32_t len)
{
    uint8_t buf[12];
    // lba
    buf[0] = (uint8_t)(lba & 0xFF);
    buf[1] = (uint8_t)((lba >> 8) & 0xFF);
    buf[2] = (uint8_t)((lba >> 16) & 0xFF);
    buf[3] = (uint8_t)((lba >> 24) & 0xFF);
    // offset
    buf[4] = (uint8_t)(offset & 0xFF);
    buf[5] = (uint8_t)((offset >> 8) & 0xFF);
    buf[6] = (uint8_t)((offset >> 16) & 0xFF);
    buf[7] = (uint8_t)((offset >> 24) & 0xFF);
    // len
    buf[8]  = (uint8_t)(len & 0xFF);
    buf[9]  = (uint8_t)((len >> 8) & 0xFF);
    buf[10] = (uint8_t)((len >> 16) & 0xFF);
    buf[11] = (uint8_t)((len >> 24) & 0xFF);
    send_frame(U_EMPTY, buf, sizeof(buf));
    Serial.printf("[MSC] send_empty_frame: lba=%u, offset=%u, len=%u (all zero)\n", lba, offset, len);
}

static void handle_read_command(const uint8_t *cmd, uint8_t len)
{
    // 固定协议：12B 小端结构
    // struct { uint32_t lba; uint32_t offset; uint32_t len; } __attribute__((packed));
    if (g_msc_dev == NULL || g_sector_size == 0) return;
    if (len != 12) return;  // 仅支持 12 字节版本

    // 解析参数（小端）
    uint32_t lba    = (uint32_t)cmd[0] | ((uint32_t)cmd[1] << 8) | ((uint32_t)cmd[2] << 16) | ((uint32_t)cmd[3] << 24);
    uint32_t offset = (uint32_t)cmd[4] | ((uint32_t)cmd[5] << 8) | ((uint32_t)cmd[6] << 16) | ((uint32_t)cmd[7] << 24);
    uint32_t rd_len = (uint32_t)cmd[8] | ((uint32_t)cmd[9] << 8) | ((uint32_t)cmd[10] << 16) | ((uint32_t)cmd[11] << 24);

    // 添加打印消息
     Serial.printf("[CMD] lba=%u, offset=%u, rd_len=%u\n", lba, offset, rd_len);

    // 计算实际的物理扇区范围
    uint32_t start_sector = lba + (offset / g_sector_size);
    uint32_t start_offset = offset % g_sector_size;
    uint32_t end_offset = start_offset + rd_len;
    uint32_t sectors_needed = (end_offset + g_sector_size - 1) / g_sector_size;

    //debug_bby Serial.printf("[DEBUG] start_sector=%u, start_offset=%u, sectors_needed=%u\n", 
    //               start_sector, start_offset, sectors_needed);

    if (sectors_needed == 0) return;

    uint8_t *sec_buf = (uint8_t*) malloc(g_sector_size * sectors_needed);
    if (!sec_buf) return;

    // 读取所有需要的扇区
    for (uint32_t i = 0; i < sectors_needed; i++) {
        esp_err_t err = msc_host_read_sector(g_msc_dev, start_sector + i, 
                                            sec_buf + (i * g_sector_size), g_sector_size);
        if (err != ESP_OK) {
            Serial.printf("[ERROR] Failed to read sector %u\n", start_sector + i);
            free(sec_buf);
            return;
        }
        Serial.printf("[READ] Read sector %u successfully\n", start_sector + i);
    }

    // 全 0 判断逻辑
    bool all_zero = true;
    for (uint32_t i = 0; i < rd_len; ++i) {
        if (sec_buf[start_offset + i] != 0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        send_empty_frame(lba, offset, rd_len);
    } else {
        // 直接发送请求的数据段（从 start_offset 开始，长度为 rd_len）
        send_readcontent_frame(lba, offset, rd_len, sec_buf + start_offset);
    }

    free(sec_buf);
}

// ------------------------------------------------------------
// Write helpers
// ------------------------------------------------------------
static void send_writedone_frame(uint32_t lba, uint32_t offset, uint32_t wr_len, uint32_t status)
{
    // payload: lba(4) + offset(4) + len(4) + status(4)
    uint8_t buf[16];
    size_t p = 0;
    // lba
    buf[p++] = (uint8_t)(lba & 0xFF);
    buf[p++] = (uint8_t)((lba >> 8) & 0xFF);
    buf[p++] = (uint8_t)((lba >> 16) & 0xFF);
    buf[p++] = (uint8_t)((lba >> 24) & 0xFF);
    // offset
    buf[p++] = (uint8_t)(offset & 0xFF);
    buf[p++] = (uint8_t)((offset >> 8) & 0xFF);
    buf[p++] = (uint8_t)((offset >> 16) & 0xFF);
    buf[p++] = (uint8_t)((offset >> 24) & 0xFF);
    // len
    buf[p++] = (uint8_t)(wr_len & 0xFF);
    buf[p++] = (uint8_t)((wr_len >> 8) & 0xFF);
    buf[p++] = (uint8_t)((wr_len >> 16) & 0xFF);
    buf[p++] = (uint8_t)((wr_len >> 24) & 0xFF);
    // status
    buf[p++] = (uint8_t)(status & 0xFF);
    buf[p++] = (uint8_t)((status >> 8) & 0xFF);
    buf[p++] = (uint8_t)((status >> 16) & 0xFF);
    buf[p++] = (uint8_t)((status >> 24) & 0xFF);

    send_frame(U_A2B_WRITEDONE, buf, (uint8_t)sizeof(buf));
    Serial.printf("[WRITE] Done: lba=%u, offset=%u, len=%u, status=%u\n", lba, offset, wr_len, status);
}

/**
 * @brief Handle U_B2A_WRITE command
 *
 * Payload format (little-endian), single-frame write:
 *   struct {
 *     uint32_t lba;     // starting LBA
 *     uint32_t offset;  // byte offset within starting LBA
 *     uint32_t len;     // data length that follows in this same frame
 *     uint8_t  data[];  // len bytes
 *   } __attribute__((packed));
 *
 * Note:
 *  - This handler writes exactly the bytes carried by this single frame.
 *  - Upper-layer may send multiple U_B2A_WRITE frames back-to-back to cover larger writes.
 *  - For partial-sector writes, we do read-modify-write to preserve untouched bytes.
 */
static void handle_write_command(const uint8_t *cmd, uint8_t total_len)
{
    if (g_msc_dev == NULL || g_sector_size == 0) return;
    if (total_len < 12) return;  // need at least header

    // Parse header
    uint32_t lba    = (uint32_t)cmd[0] | ((uint32_t)cmd[1] << 8) | ((uint32_t)cmd[2] << 16) | ((uint32_t)cmd[3] << 24);
    uint32_t offset = (uint32_t)cmd[4] | ((uint32_t)cmd[5] << 8) | ((uint32_t)cmd[6] << 16) | ((uint32_t)cmd[7] << 24);
    uint32_t wr_len = (uint32_t)cmd[8] | ((uint32_t)cmd[9] << 8) | ((uint32_t)cmd[10] << 16) | ((uint32_t)cmd[11] << 24);

    if (wr_len == 0) { send_writedone_frame(lba, offset, wr_len, 0); return; }
    if ((uint32_t)(total_len - 12) < wr_len) {
        // Frame doesn't carry all declared data
        Serial.printf("[WRITE][ERR] Frame len %u smaller than declared data %u\n", (unsigned)total_len, (unsigned)wr_len);
        send_writedone_frame(lba, offset, wr_len, 1);
        return;
    }

    const uint8_t *data = cmd + 12;

    // Compute sector window
    uint32_t start_sector    = lba + (offset / g_sector_size);
    uint32_t start_off_in_se = offset % g_sector_size;
    uint32_t end_offset      = start_off_in_se + wr_len;
    uint32_t sectors_needed  = (end_offset + g_sector_size - 1) / g_sector_size;

    Serial.printf("[WRITE] lba=%u, offset=%u, len=%u -> start_sector=%u, start_off=%u, sectors=%u\n",
                  lba, offset, wr_len, start_sector, start_off_in_se, sectors_needed);

    esp_err_t err = ESP_OK;
    // For read-modify-write we allocate a staging buffer that covers all needed sectors
    uint32_t buf_bytes = sectors_needed * g_sector_size;
    uint8_t *staging = (uint8_t *)malloc(buf_bytes);
    if (!staging) {
        Serial.printf("[WRITE][ERR] malloc %u bytes failed\n", (unsigned)buf_bytes);
        send_writedone_frame(lba, offset, wr_len, 2);
        return;
    }

    // Read existing sectors first (to preserve bytes not covered by this frame)
    for (uint32_t i = 0; i < sectors_needed; i++) {
        err = msc_host_read_sector(g_msc_dev, start_sector + i, staging + i * g_sector_size, g_sector_size);
        if (err != ESP_OK) {
            Serial.printf("[WRITE][ERR] read back sector %u failed\n", start_sector + i);
            free(staging);
            send_writedone_frame(lba, offset, wr_len, 3);
            return;
        }
    }

    // Patch in new data
    memcpy(staging + start_off_in_se, data, wr_len);

    // Write back each affected sector
    for (uint32_t i = 0; i < sectors_needed; i++) {
        err = msc_host_write_sector(g_msc_dev, start_sector + i, staging + i * g_sector_size, g_sector_size);
        if (err != ESP_OK) {
            Serial.printf("[WRITE][ERR] write sector %u failed\n", start_sector + i);
            free(staging);
            send_writedone_frame(lba, offset, wr_len, 4);
            return;
        }
    }

    free(staging);
    // Success
    send_writedone_frame(lba, offset, wr_len, 0);
}


//bby
static void msc_task(void *args)
{
    msc_host_device_handle_t msc_device = NULL;
    msc_host_vfs_handle_t vfs_handle = NULL;

    for (;;) {
        app_message_t msg;
        if (!xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            continue;
        }

        if (msg.id == APP_DEVICE_CONNECTED) {
            if (dev_present) {
                ESP_LOGW(TAG, "MSC Example handles only one device at a time");
                continue;
            }
            dev_present = true;

            // 1) 安装设备并挂载到 VFS
            ESP_ERROR_CHECK(msc_host_install_device(msg.data.new_dev_address, &msc_device));
            g_msc_dev = msc_device;
            const esp_vfs_fat_mount_config_t mount_config = {
                .format_if_mount_failed   = false,
                .max_files                = 3,
                .allocation_unit_size     = 8192,
            };
            ESP_ERROR_CHECK(msc_host_vfs_register(msc_device, MNT_PATH, &mount_config, &vfs_handle));

            // 2) 打印设备信息/描述符
            msc_host_device_info_t info;
            ESP_ERROR_CHECK(msc_host_get_device_info(msc_device, &info));
            msc_host_print_descriptors(msc_device);
            print_device_info(&info);

            // 3) 列目录 + 4) 文件操作 + 5) 速度测试
            ESP_LOGI(TAG, "ls command output:");
            struct dirent *d;
            DIR *dh = opendir(MNT_PATH);
            assert(dh);
            while ((d = readdir(dh)) != NULL) {
                printf("%s\n", d->d_name);
            }
            closedir(dh);

            // bby file_operations();
            // speed_test();

            ESP_LOGI(TAG, "Example finished, you can disconnect the USB flash drive");
        }

        if ((msg.id == APP_DEVICE_DISCONNECTED) || (msg.id == APP_QUIT)) {
            if (dev_present) {
                send_frame(U_A2B_REMOVED, NULL, 0);
                dev_present = false;
                if (vfs_handle) {
                    ESP_ERROR_CHECK(msc_host_vfs_unregister(vfs_handle));
                    vfs_handle = NULL;
                }
                if (msc_device) {
                    ESP_ERROR_CHECK(msc_host_uninstall_device(msc_device));
                    msc_device = NULL;
                }
            }
            if (msg.id == APP_QUIT) {
                // 触发 MSC driver 卸载；USB 库卸载由 usb_task 根据 NO_CLIENTS 事件完成
                ESP_ERROR_CHECK(msc_host_uninstall());
                break;  // 跳出任务主循环，做收尾并结束任务
            }
        }
        if (msg.id == APP_COMMAND) {
            // 解析来自上游的命令
            if (msg.data.cmd.type == U_B2A_READ) {
                handle_read_command(msg.data.cmd.data, msg.data.cmd.len);
            } else if (msg.data.cmd.type == U_B2A_WRITE) {
                handle_write_command(msg.data.cmd.data, msg.data.cmd.len);
            }
        }
    }

    ESP_LOGI(TAG, "msc_task exiting");
    vTaskDelete(NULL);
}

//bby
static void uart_task(void *arg) {
    while (1) {
        uint8_t type, payload[256], len;
        if (read_frame(type, payload, len, sizeof(payload))) { // ← 改1：不再取地址
            // 添加打印消息
            //Serial.printf("[UART] Received: type=0x%02X, len=%d\n", type, len);
            app_message_t msg{};
            msg.id = APP_COMMAND;                              // ← 改2：有这个枚举
            msg.data.cmd.type = type;               // 记录命令类型
            memcpy(msg.data.cmd.data, payload, len);           // ← 改3：使用 cmd 成员
            msg.data.cmd.len = len;
            if (type == U_B2A_WRITE) {
                Serial.printf("[UART] RX WRITE frame, len=%u\n", len);
            }
            xQueueSend(app_queue, &msg, portMAX_DELAY);
        }
        vTaskDelay(1);
    }
}



void app_main(void) {
  BaseType_t task_created;
  ESP_LOGI(TAG, "HID+MSC Host starting");

  // (1) 启动 USB Host 库 & 事件循环（只此一次）
  task_created =
      xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096,
                              xTaskGetCurrentTaskHandle(), 2, NULL, 0);
  assert(task_created == pdTRUE);

  // 等待 usb_lib_task 完成安装
  ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

  // (2) 安装 HID Host（一次）并启动 HID 事件任务
  const hid_host_driver_config_t hid_host_driver_config = {
      .create_background_task = true,
      .task_priority = 5,
      .stack_size = 4096,
      .core_id = 0,
      .callback = hid_host_device_callback,
      .callback_arg = NULL};

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
  user_shutdown = false;

  task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
  assert(task_created == pdTRUE);

  // (3) MSC 应用：创建队列、安装 MSC 驱动、创建 msc_task 与 uart_task
  app_queue = xQueueCreate(5, sizeof(app_message_t));
  assert(app_queue);

  // 只安装 MSC 驱动（由 usb_task 完成），不重复安装 usb_host
  task_created = xTaskCreate(usb_task, "usb_task", 4096, NULL, 2, NULL);
  assert(task_created == pdTRUE);

  // BOOT 键中断：按下发 APP_QUIT
  const gpio_config_t input_pin = {
      .pin_bit_mask = BIT64(APP_QUIT_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&input_pin));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
  ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

  // 启动 MSC 应用任务与 UART 任务
  task_created = xTaskCreate(msc_task, "msc_task", 5 * 1024, NULL, 3, NULL);
  assert(task_created == pdTRUE);

  task_created = xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL);
  assert(task_created == pdTRUE);

  ESP_LOGI(TAG, "System init done. Waiting for devices...");
}






//----------------------------------------------------------------------------
// 主程序入口
//----------------------------------------------------------------------------

// 选择引脚
static constexpr int PIN_MODE_SEL = 5;
// true  -> 运行 app_main()（host/passthrough）
// false -> 运行 device 模式（HID/MSC）
static bool g_run_host = false;
// 防止误触发的标志
static bool g_appmain_started = false;
static bool g_device_inited   = false;

void setup() {
  // 串口与选择引脚
  Serial.begin(460800);
  delay(200);
  pinMode(PIN_MODE_SEL, INPUT_PULLUP);   // 若外部下拉=设备模式；悬空/上拉=host模式

  // UART 初始化（两端都需要）
  uart2_init();

  // 读取选择（上电一次判定；如果需要运行时切换，可在 loop 里扩展）
  g_run_host = (digitalRead(PIN_MODE_SEL) == HIGH);

  if (g_run_host) {
    // ---------- Host / Passthrough 分支 ----------
    Serial.println("ESP32-A: HID+MSC passthrough (HOST) selected by GPIO5=HIGH");
    // 只启动一次 app_main，内部会创建 FreeRTOS 任务、usb_host_install 等
    if (!g_appmain_started) {
      g_appmain_started = true;
      app_main();
    }
    // 注意：后续逻辑在 app_main() 创建的任务里跑；loop() 留空即可
  } else {
    // ---------- Device (HID/MSC) 分支 ----------
    Serial.println("USB Device mode (HID/MSC) selected by GPIO5=LOW");
    device_usb_init();
    g_device_inited = true;
    Serial.println("USB initialization done (device)");
  }
}

void loop() {
  if (g_run_host) {
    // Host/passthrough 模式：所有工作在线程里，loop 留空避免误触发
    // 也可适当让出 CPU：
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }

  // 设备模式：保持你原有的逻辑
  // 未就绪或者模式为 NONE -> 重新初始化
  if (!g_usb_ready || g_Mode == NONE) {
    if (!g_device_inited || !g_usb_ready) {
      device_usb_init();
      g_device_inited = true;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  // HID 模式：串口 -> HID 转发
  if (g_Mode == HID) {
    device_mouse_and_keyboard();
    // 函数内部通常会阻塞/轮询；这里不额外延时
  }
  // MSC 模式：电脑通过回调驱动，无需主动干预；仅监测是否需要重新初始化
  else if (g_Mode == USTICK) {
    if (check_reinit_needed()) {
      Close_Ustick();   // 触发设备侧清理，下一轮会走初始化分支
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}