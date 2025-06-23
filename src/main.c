#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include "esp_err.h"
#include "esp_vfs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "math.h"

// Параметры I2C и MPU6050
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   21
#define I2C_MASTER_SCL_IO   22
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_TIMEOUT_MS      1000
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define ACCEL_SCALE_FACTOR  16384.0f  // LSB/g для диапазона ±2g

// Калибровочные коэффициенты (пример)
static const float bias[3]  = { 0.028299f, -0.021181f,  0.061112f };
static const float Q[3][3]  = { { 0.99451858f, -0.00161528f, -0.00204506f },
                                { -0.00161528f,  1.00018706f,  0.00621883f },
                                { -0.00204506f,  0.00621883f,  1.01006318f } };
static const float scale[3] = { 1.013409f, 0.997285f, 0.994075f };

// Инициализация SPIFFS (без форматирования при каждом старте)
static esp_err_t init_filesystem(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    // Проверка целостности SPIFFS после монтирования:
    esp_err_t res = esp_spiffs_check(NULL);
    if (res != ESP_OK) {
        ESP_LOGW("SPIFFS", "Проверка SPIFFS завершилась ошибкой: %s", esp_err_to_name(res));
    }
    return ESP_OK;
}

// Инициализация Wi-Fi SoftAP (SSID и пароль заданы)
static void init_wifi_softap(void) {
    // NVS необходим для работы Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Параметры точки доступа
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = sizeof("ESP32_AP") - 1,
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Вывод информации о запуске AP
    ESP_LOGI("WIFI", "SoftAP запущен: SSID=%s, PASS=%s", wifi_config.ap.ssid, wifi_config.ap.password);
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
        ESP_LOGI("WIFI", "AP IP адрес: " IPSTR, IP2STR(&ip_info.ip));
    }
}

// Настройка I2C для MPU6050
static esp_err_t i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Запись в регистр MPU6050 (вспомогательная функция)
static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Чтение всех регистрационных данных датчика (14 байт: акселерометр + гироскоп)
static esp_err_t mpu6050_read_raw(uint8_t *data_buffer) {
    // Команда чтения 14 байт начиная с регистра ACCEL_XOUT_H
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_ACCEL_XOUT_H, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        return err;
    }
    // Повторный старт для чтения 14 байт
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    for (int i = 0; i < 14; ++i) {
        i2c_master_read_byte(cmd, &data_buffer[i], (i == 13) ? I2C_MASTER_NACK : I2C_MASTER_ACK);
    }
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

// HTTP-обработчик GET /accel.csv – отправляет содержимое файла
static esp_err_t accel_csv_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/csv");
    FILE *f = fopen("/spiffs/accel.csv", "r");
    if (f == NULL) {
        // Файл ещё не создан или недоступен
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_OK;
    }
    char buf[128];
    while (fgets(buf, sizeof(buf), f)) {
        httpd_resp_send_chunk(req, buf, strlen(buf));
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);  // завершение ответа
    return ESP_OK;
}

// Запуск HTTP-сервера и регистрация URI
static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        static httpd_uri_t uri_handler = {
            .uri      = "/accel.csv",
            .method   = HTTP_GET,
            .handler  = accel_csv_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_handler);
        ESP_LOGI("HTTP", "HTTP-сервер запущен (файл /accel.csv доступен)");
    }
}

// Главная функция приложения
void app_main(void) {
    // 1. Старт SPIFFS (монтаж без форматирования)
    init_filesystem();
    ESP_LOGI("SPIFFS", "Файловая система готова");

    // 2. Старт Wi-Fi в режиме SoftAP
    init_wifi_softap();

    // 3. Запуск HTTP-сервера
    start_webserver();

    // 4. Инициализация I2C и пробуждение MPU6050
    ESP_ERROR_CHECK(i2c_init());
    vTaskDelay(pdMS_TO_TICKS(200));
    mpu6050_write_byte(REG_PWR_MGMT_1, 0x00);  // вывести MPU6050 из sleep-режима

    // Переменные состояния записи
    bool recording = false;
    uint64_t start_time_us = 0;
    FILE *f = NULL;
    int sample_count = 0;
    static int freeze_counter = 0;

    // 5. Основной цикл чтения датчика (каждые 10 мс)
    while (1) {

        uint8_t raw[14];
        if (mpu6050_read_raw(raw) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;  // повторить попытку чтения через 10мс
        }
        // Конвертация сырых данных
        int16_t raw_ax = (raw[0] << 8) | raw[1];
        int16_t raw_ay = (raw[2] << 8) | raw[3];
        int16_t raw_az = (raw[4] << 8) | raw[5];
        // Приведение к физическим величинам (g)
        float ax = raw_ax / ACCEL_SCALE_FACTOR;
        float ay = raw_ay / ACCEL_SCALE_FACTOR;
        float az = raw_az / ACCEL_SCALE_FACTOR;
        // Применение калибровки: bias, матрица Q, scale
        float v0 = ax - bias[0], v1 = ay - bias[1], v2 = az - bias[2];
        float y0 = Q[0][0]*v0 + Q[1][0]*v1 + Q[2][0]*v2;
        float y1 = Q[0][1]*v0 + Q[1][1]*v1 + Q[2][1]*v2;
        float y2 = Q[0][2]*v0 + Q[1][2]*v1 + Q[2][2]*v2;
        y0 *= scale[0];  y1 *= scale[1];  y2 *= scale[2];
        float cal_ax = Q[0][0]*y0 + Q[0][1]*y1 + Q[0][2]*y2;
        float cal_ay = Q[1][0]*y0 + Q[1][1]*y1 + Q[1][2]*y2;
        float cal_az = Q[2][0]*y0 + Q[2][1]*y1 + Q[2][2]*y2;
        // Модуль полного ускорения (g_total)
        float accel_total = sqrtf(cal_ax*cal_ax + cal_ay*cal_ay + cal_az*cal_az);

        bool is_not_static_point = (
            fabs(cal_ax + 0.028f) > 0.001f ||
            fabs(cal_ay - 0.020f) > 0.001f ||
            fabs(cal_az + 0.062f) > 0.001f
        );

        float tolerance = 0.001f;
        if (fabsf(cal_ax + 0.028f) < tolerance &&
            fabsf(cal_ay - 0.020f) < tolerance &&
            fabsf(cal_az + 0.062f) < tolerance) {
            freeze_counter++;
            if (freeze_counter >= 5) {
                esp_restart();
            }
        } else {
            freeze_counter = 0;
        }

        // 6. Обнаружение свободного падения и старт записи


        if (!recording && accel_total < 0.1f && is_not_static_point) {
            ESP_LOGI("SPIFFS", "=== НАЧАЛО ЗАПИСИ CSV ===");
            // Открыть/пересоздать файл для записи
            f = fopen("/spiffs/accel.csv", "w");
            if (f == NULL) {
                ESP_LOGE("SPIFFS", "Ошибка создания файла accel.csv");
            } else {
                // Начало записи: сброс счётчика и отметка времени
                recording = true;
                sample_count = 0;
                start_time_us = esp_timer_get_time();
                // Записать первую выборку (t=0) сразу
                fprintf(f, "0,%.3f,%.3f,%.3f\n", cal_ax, cal_ay, cal_az);
                sample_count++;
            }
        }

        // 7. Запись данных в CSV каждые 10мс (пока не истечёт 4с)
        if (recording && f != NULL) {
            uint64_t now_us = esp_timer_get_time();
            uint64_t elapsed_ms = (now_us - start_time_us) / 1000ULL;
            if (elapsed_ms <= 4000ULL) {
                // Записать очередную строку CSV
                fprintf(f, "%llu,%.3f,%.3f,%.3f\n",
                        (unsigned long long)elapsed_ms, cal_ax, cal_ay, cal_az);
                // Каждые ~40мс сбрасывать данные на флеш
                if (sample_count % 4 == 0) {
                    fflush(f);
                }
                sample_count++;
            } else {
                // 4 секунды прошли – остановить запись
                ESP_LOGI("SPIFFS", "=== ОКОНЧАНИЕ ЗАПИСИ ===");
                fclose(f);
                recording = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // задержка 10 мс до следующего измерения
    }
}
