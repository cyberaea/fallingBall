#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
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
#include <sys/socket.h>
#include <arpa/inet.h>

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000
#define MPU6050_ADDR                0x68
#define MPU6050_PWR_MGMT_1_REG      0x6B
#define MPU6050_ACCEL_XOUT_H_REG    0x3B
#define ACCEL_SCALE_FACTOR          16384.0

static const char *TAG = "MPU6050_LOG";
static SemaphoreHandle_t spiffs_mutex = NULL;
static volatile bool recording = false;
static SemaphoreHandle_t recording_mutex = NULL;

// Прототипы функций
static void wifi_init_softap(void);
static void sensor_task(void *pvParameters);
static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

static void i2c_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus for devices…");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, " -- Device found at 0x%02X", addr);
        }
    }
}


static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = { reg_addr, data };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write(cmd, write_buf, sizeof(write_buf), true);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    ret |= i2c_master_write_byte(cmd, reg_addr, true);
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_init(void) {
    return mpu6050_write_reg(MPU6050_PWR_MGMT_1_REG, 0x00);
}

static esp_err_t init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS init failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// HTTP-обработчик
#include <sys/socket.h> // Добавьте этот заголовок

static esp_err_t log_file_get_handler(httpd_req_t *req) {
    // Получаем IP клиента через дескриптор сокета
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd < 0) {
        ESP_LOGE(TAG, "Failed to get socket descriptor");
        return ESP_FAIL;
    }

    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    if (getpeername(sockfd, (struct sockaddr*)&addr, &addr_len) == 0) {
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "Request from IP: %s", ip_str);
    } else {
        ESP_LOGE(TAG, "Failed to get client IP");
    }

    // Логика работы с файлом
    if (xSemaphoreTake(spiffs_mutex, pdMS_TO_TICKS(2000))) {
        FILE *f = fopen("/spiffs/accel_log.txt", "r");
        if (!f) {
            ESP_LOGE(TAG, "File not found");
            httpd_resp_send_404(req);
            xSemaphoreGive(spiffs_mutex);
            return ESP_FAIL;
        }

        char buffer[128];
        size_t bytes_read;
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), f)) > 0) {
            httpd_resp_send_chunk(req, buffer, bytes_read);
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

static httpd_uri_t log_uri = {
    .uri = "/log",
    .method = HTTP_GET,
    .handler = log_file_get_handler,
    .user_ctx = NULL
};

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_open_sockets = 5;

    httpd_handle_t server = NULL;
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server!");
        return NULL;
    }

    if (httpd_register_uri_handler(server, &log_uri) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register URI handler");
        httpd_stop(server);
        return NULL;
    }

    return server;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected: MAC=%02x:%02x:%02x:%02x:%02x:%02x",
                event->mac[0], event->mac[1], event->mac[2],
                event->mac[3], event->mac[4], event->mac[5]);
    }
}

static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .password = "esp32password",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false
            }
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    
    esp_netif_dhcps_stop(ap_netif);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
    
    ESP_LOGI(TAG, "WiFi AP Started. SSID: %s, IP: " IPSTR, 
           wifi_config.ap.ssid, IP2STR(&ip_info.ip));
}

static void sensor_task(void *pvParameters) {
    FILE *f = NULL;
    uint32_t start_time = 0;
    const float GYRO_SCALE = 131.0f; // Для диапазона ±250 °/с
    
    while (1) {
        uint8_t data[14];
        esp_err_t read_err = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H_REG, data, sizeof(data));
        if (read_err != ESP_OK) {
            ESP_LOGE(TAG, "MPU6050 read failed: %s", esp_err_to_name(read_err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }


        // Акселерометр
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        float accel_total = sqrt(
            pow(ax / ACCEL_SCALE_FACTOR, 2) + 
            pow(ay / ACCEL_SCALE_FACTOR, 2) + 
            pow(az / ACCEL_SCALE_FACTOR, 2)
        );

        // Гироскоп
        int16_t gx = (data[8] << 8) | data[9];
        int16_t gy = (data[10] << 8) | data[11];
        int16_t gz = (data[12] << 8) | data[13];
        float gyro_x = gx / GYRO_SCALE;
        float gyro_y = gy / GYRO_SCALE;
        float gyro_z = gz / GYRO_SCALE;

        // Логика записи
        if (xSemaphoreTake(recording_mutex, pdMS_TO_TICKS(1000))) {
            if (accel_total > 2.5f && !recording) {
                recording = true;
                start_time = esp_timer_get_time();
                ESP_LOGI(TAG, "Recording started");

                if (xSemaphoreTake(spiffs_mutex, pdMS_TO_TICKS(1000))) {
                    f = fopen("/spiffs/accel_log.txt", "w");
                    if (!f) {
                        ESP_LOGE(TAG, "Failed to open file");
                    } else {
                        fprintf(f, "{\"start_time\": %" PRIu32 "}\n", start_time);
                        fflush(f);
                    }
                    xSemaphoreGive(spiffs_mutex);
                }
            }
            xSemaphoreGive(recording_mutex);
        }

        // Запись данных
        if (recording) {
            uint32_t current_time = esp_timer_get_time();
            if ((current_time - start_time) <= 30000000) {
                char log_line[256]; // Увеличенный буфер
                snprintf(
                    log_line, sizeof(log_line),
                    "{\"t\": %" PRIu32 
                    ", \"ax\": %.3f, \"ay\": %.3f, \"az\": %.3f" 
                    ", \"gx\": %.2f, \"gy\": %.2f, \"gz\": %.2f}\n",
                    current_time - start_time,
                    ax / ACCEL_SCALE_FACTOR,
                    ay / ACCEL_SCALE_FACTOR,
                    az / ACCEL_SCALE_FACTOR,
                    gyro_x, gyro_y, gyro_z
                );

                if (xSemaphoreTake(spiffs_mutex, pdMS_TO_TICKS(1000))) {
                    if (f) {
                        fprintf(f, "%s", log_line);
                        fflush(f);
                    }
                    xSemaphoreGive(spiffs_mutex);
                }
            } else {
                recording = false;
                if (f) {
                    fclose(f);
                    f = NULL;
                }
                ESP_LOGI(TAG, "Recording stopped");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {

    // Инициализация мьютексов
    spiffs_mutex = xSemaphoreCreateMutex();
    recording_mutex = xSemaphoreCreateMutex();
    if (!spiffs_mutex || !recording_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return;
    }

    // Инициализация периферии
    ESP_ERROR_CHECK(i2c_master_init());
    i2c_scan();
    esp_err_t err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "MPU6050 successfully initialized");
    }


    ESP_ERROR_CHECK(init_spiffs());

    // Настройка Wi-Fi и HTTP-сервера
    wifi_init_softap();
    if (start_webserver() == NULL) {
        ESP_LOGE(TAG, "Web server failed");
        return;
    }

    // Запуск задачи
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "System initialized");
}