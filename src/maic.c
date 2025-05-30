#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <errno.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"

#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_SDA_IO     21
#define I2C_MASTER_SCL_IO     22
#define I2C_MASTER_FREQ_HZ    100000
#define I2C_MASTER_TIMEOUT_MS 1000

#define MPU6050_ADDR              0x68
#define MPU6050_PWR_MGMT_1_REG    0x6B
#define MPU6050_ACCEL_XOUT_H_REG  0x3B
#define ACCEL_SCALE_FACTOR        16384.0f

// Единственная точка входа для лога
#define LOG_FILE   "/spiffs/accel_log.txt"

static const char *TAG = "MPU6050_LOG";
static SemaphoreHandle_t spiffs_mutex;
static SemaphoreHandle_t recording_mutex;
static volatile bool recording = false;

// Прототипы
static void               i2c_scan(void);
static esp_err_t          i2c_master_init(void);
static esp_err_t          mpu6050_write_reg(uint8_t, uint8_t);
static esp_err_t          mpu6050_read_bytes(uint8_t, uint8_t*, size_t);
static esp_err_t          mpu6050_init(void);
static esp_err_t          init_spiffs(void);
static esp_err_t          start_webserver(void);
static void               sensor_task(void*);
static esp_err_t          log_handler(httpd_req_t*);
static void               wifi_event_handler(void*, esp_event_base_t, int32_t, void*);
static void               wifi_init_softap(void);

// ——— init SPIFFS ———
static esp_err_t init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = "spiffs",     // из partitions.csv
        .max_files              = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total=%u bytes, used=%u bytes", total, used);
    } else {
        ESP_LOGE(TAG, "SPIFFS info failed: %s", esp_err_to_name(ret));
    }
    return ESP_OK;
}

// ——— HTTP /log ———
static esp_err_t log_handler(httpd_req_t *req) {
    if (xSemaphoreTake(spiffs_mutex, pdMS_TO_TICKS(2000))) {
        FILE *f = fopen(LOG_FILE, "r");
        if (!f) {
            ESP_LOGE(TAG, "fopen(%s) failed: errno=%d", LOG_FILE, errno);
            httpd_resp_send_404(req);
            xSemaphoreGive(spiffs_mutex);
            return ESP_FAIL;
        }
        char buf[128];
        size_t r;
        while ((r = fread(buf,1,sizeof(buf),f)) > 0) {
            httpd_resp_send_chunk(req, buf, r);
        }
        fclose(f);
        httpd_resp_send_chunk(req, NULL, 0);
        xSemaphoreGive(spiffs_mutex);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "SPIFFS mutex timeout");
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

static const httpd_uri_t log_uri = {
    .uri      = "/log",
    .method   = HTTP_GET,
    .handler  = log_handler,
    .user_ctx = NULL
};

static esp_err_t start_webserver(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 5;
    httpd_handle_t serv = NULL;
    if (httpd_start(&serv, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return ESP_FAIL;
    }
    httpd_register_uri_handler(serv, &log_uri);
    ESP_LOGI(TAG, "HTTP server running");
    return ESP_OK;
}

// … (Wi-Fi I2C функции остаются без изменений) …

// ——— Задача сенсора ———
static void sensor_task(void* pvParameters) {
    FILE *f = NULL;
    uint32_t start_us = 0;
    const float GYRO_SCALE = 131.0f;

    while (1) {
        // … считываем MPU6050 в массив d[14] …

        float ax = ((int16_t)((d[0]<<8)|d[1]))/ACCEL_SCALE_FACTOR;
        float ay = ((int16_t)((d[2]<<8)|d[3]))/ACCEL_SCALE_FACTOR;
        float az = ((int16_t)((d[4]<<8)|d[5]))/ACCEL_SCALE_FACTOR;
        float tot = sqrtf(ax*ax + ay*ay + az*az);

        // Диагностика точки монтирования
        struct stat st;
        if (stat("/spiffs", &st) == 0) {
            ESP_LOGI(TAG, "/spiffs exists, mode=0%o", st.st_mode);
        } else {
            ESP_LOGE(TAG, "stat /spiffs failed: errno=%d", errno);
        }
        DIR *d = opendir("/spiffs");
        if (d) {
            struct dirent *ent;
            while ((ent = readdir(d)) != NULL) {
                ESP_LOGI(TAG, "spiffs entry: %s", ent->d_name);
            }
            closedir(d);
        } else {
            ESP_LOGE(TAG, "opendir /spiffs failed: errno=%d", errno);
        }

        if (xSemaphoreTake(recording_mutex, pdMS_TO_TICKS(1000))) {
            if (tot > 2.5f && !recording) {
                recording = true;
                start_us = esp_timer_get_time();
                ESP_LOGI(TAG, "Recording started");

                // собственно запись
                if (xSemaphoreTake(spiffs_mutex, pdMS_TO_TICKS(1000))) {
                    ESP_LOGI(TAG, "Opening %s for write", LOG_FILE);
                    f = fopen(LOG_FILE, "w");
                    if (!f) {
                        ESP_LOGE(TAG, "fopen(%s) failed: errno=%d", LOG_FILE, errno);
                    } else {
                        fprintf(f, "{\"start_time\":%"PRIu32"}\n", start_us);
                        fflush(f);
                    }
                    xSemaphoreGive(spiffs_mutex);
                }
            }
            xSemaphoreGive(recording_mutex);
        }

        // … остальная часть записи до 30 секунд …

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    spiffs_mutex    = xSemaphoreCreateMutex();
    recording_mutex = xSemaphoreCreateMutex();
    if (!spiffs_mutex || !recording_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return;
    }
    ESP_ERROR_CHECK(i2c_master_init());
    i2c_scan();
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_ERROR_CHECK(init_spiffs());
    wifi_init_softap();
    ESP_ERROR_CHECK(start_webserver());
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "System initialized");
}
