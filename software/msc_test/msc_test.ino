/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <Arduino.h>
#include "uart.h"   // 提供：uart2_init(), send_frame(uint8_t type, const uint8_t* payload, uint8_t len)
#include "usb.h"    // 可选：若后续要把帧转给 USB HID 设备

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "msc_host.h"
#include "msc_host_vfs.h"
#include "ffconf.h"
#include "errno.h"
#include "driver/gpio.h"

static const char *TAG = "example";
#define MNT_PATH         "/usb"     // Path in the Virtual File System, where the USB flash drive is going to be mounted
#define APP_QUIT_PIN     GPIO_NUM_0 // BOOT button on most boards
#define BUFFER_SIZE      4096       // The read/write performance can be improved with larger buffer for the cost of RAM, 4kB is enough for most usecases

// IMPORTANT NOTE
// MSC Class Driver is not fully support connecting devices through external Hub.
// TODO: Remove this line after MSC Class Driver will support it
static bool dev_present = false;

static uint32_t g_sector_size = 512;                 // 从设备信息更新
static msc_host_device_handle_t g_msc_dev = NULL;    // 当前 MSC 设备句柄
static const uint8_t CHUNK_DATA_MAX = 240;           // 单帧分片最大数据长度（<=255-头部）

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
    const usb_host_config_t host_config = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };
    ESP_ERROR_CHECK(msc_host_install(&msc_config));

    bool has_clients = true;
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
            if (usb_host_device_free_all() == ESP_OK) {
                break;
            };
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE && !has_clients) {
            break;
        }
    }

    vTaskDelay(10); // Give clients some time to uninstall
    ESP_LOGI(TAG, "Deinitializing USB");
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

static void send_readcontent_frame(uint32_t lba, uint32_t offset, uint32_t data_len,
                                   const uint8_t* data)
{
    // 按240字节分片发送数据
    uint32_t sent = 0;
    while (sent < data_len) {
        uint32_t chunk_size = (data_len - sent > 240) ? 240 : (data_len - sent);
        
        // 计算当前chunk对应的实际LBA和扇区内偏移
        uint32_t current_absolute_offset = offset + sent;
        uint32_t current_lba = lba + (current_absolute_offset / g_sector_size);
        uint32_t current_offset_in_sector = current_absolute_offset % g_sector_size;
        
        uint8_t buf[4 + 4 + 4 + 240]; // lba(4) + offset(4) + data_length(4) + data(最大240字节)
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

        // 添加打印消息
        Serial.printf("[MSC] Sending chunk %u/%u: lba=%u, sector_offset=%u, data_length=%u (abs_offset=%u)\n", 
                      (sent/240)+1, (data_len+239)/240, current_lba, current_offset_in_sector, chunk_size, current_absolute_offset);

        // 打印完整的 send_frame 数据（协议头+数据）
        Serial.printf("[FRAME] Complete frame data (%u bytes): ", (unsigned)p);
        for (size_t i = 0; i < p; i++) {
            Serial.printf("%02X ", buf[i]);
            if ((i + 1) % 16 == 0) Serial.println(); // 每16字节换行
        }
        if (p % 16 != 0) Serial.println(); // 最后一行换行

        // 如果是 LBA=0，单独打印末尾两个字节
        if (current_lba == 0 && current_offset_in_sector + chunk_size >= 510) {
            if (chunk_size >= 2) {
                uint8_t b1 = data[sent + chunk_size - 2];
                uint8_t b2 = data[sent + chunk_size - 1];
                Serial.printf("[A] >>> LBA0 last two bytes = %02X %02X (expect 55 AA)\n", b1, b2);
            }
        }

        send_frame(U_A2B_READCONTENT, buf, (uint8_t)p);
        
        sent += chunk_size;
    }
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

    Serial.printf("[DEBUG] start_sector=%u, start_offset=%u, sectors_needed=%u\n", 
                  start_sector, start_offset, sectors_needed);

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

    // 直接发送请求的数据段（从 start_offset 开始，长度为 rd_len）
    send_readcontent_frame(lba, offset, rd_len, sec_buf + start_offset);

    free(sec_buf);
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
            xQueueSend(app_queue, &msg, portMAX_DELAY);
        }
        vTaskDelay(1);
    }
}



void app_main(void)
{
    // 1) 创建应用队列
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);

    // 2) 创建 USB 栈任务（保持原样）
    BaseType_t ok = xTaskCreate(usb_task, "usb_task", 4096, NULL, 2, NULL);
    assert(ok);

    // 3) BOOT 键中断：按下发 APP_QUIT（保持原样）
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    // 4) 安装 MSC host（仍在 usb_task 里调用 msc_host_install() 也可以；二选一，避免重复）
    // ——当前你的 msc_host_install() 在 usb_task() 里做了，这里无需重复。

    ESP_LOGI(TAG, "Waiting for USB flash drive to be connected");

    // 5) 创建 MSC 应用任务（把原 while(1) 的逻辑交给它）
    ok = xTaskCreate(msc_task, "msc_task", 5*1024, NULL, 3, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 2, NULL);//bby
    assert(ok);

    // 6) app_main 到此结束（可选择什么都不做或删除自身）
    // vTaskDelete(NULL); // 如果你希望 app_main 对应的 main_task 退出
}



void setup() {
  Serial.begin(460800);      // 调试口到电脑
  delay(200);
  uart2_init();              // 串口2：与对端板/下游链路通信
  Serial.println("ESP32-A: Device -> USB MSC test");
  app_main();
}

void loop() {

}