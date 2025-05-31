#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

// --- SPIFFS ---
#include "esp_spiffs.h"
#include "esp_err.h"
#include "esp_vfs.h"
// ------------

// --- Wi-Fi + HTTP ---
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
// ------------

#define TAG            "BASIC_TEST"
#define TAG_SPI        "SPIFFS_TEST"
#define TAG_WIFI       "WIFI_INIT"
#define TAG_HTTP       "HTTP_SERVER"

// I2C, MPU6050
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1_REG  0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B

#define ACCEL_SCALE_FACTOR      16384.0f
#define GYRO_SCALE_FACTOR       131.0f

/*--------------------------------------
   SPIFFS initialization
--------------------------------------*/
static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPI, "esp_vfs_spiffs_register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SPI, "SPIFFS total: %u, used: %u", total, used);
    } else {
        ESP_LOGE(TAG_SPI, "esp_spiffs_info failed: %s", esp_err_to_name(ret));
    }
    return ESP_OK;
}

/*--------------------------------------
   Wi-Fi SoftAP initialization
--------------------------------------*/
static void wifi_init_softap(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = strlen("ESP32_AP"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen((char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "WiFi SoftAP запущен. SSID: %s  Password: %s",
             wifi_config.ap.ssid, wifi_config.ap.password);

    esp_netif_ip_info_t ip_info;
    ret = esp_netif_get_ip_info(ap_netif, &ip_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_WIFI, "AP IP: " IPSTR, IP2STR(&ip_info.ip));
    } else {
        ESP_LOGE(TAG_WIFI, "esp_netif_get_ip_info failed: %s", esp_err_to_name(ret));
    }
}

/*--------------------------------------
   I2C, MPU6050 support
--------------------------------------*/
static esp_err_t i2c_master_init(void)
{
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

static esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t mpu6050_read_all(uint8_t *raw)
{
    esp_err_t err = ESP_FAIL;

    for (int attempt = 0; attempt < 2; attempt++) {
        // 1) Пишем адрес регистра
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 2) Пауза 5 мс перед чтением
        vTaskDelay(pdMS_TO_TICKS(5));

        // 3) Читаем 14 байт данных
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        for (size_t i = 0; i < 14; i++) {
            i2c_master_read_byte(cmd, &raw[i], (i == 13) ? I2C_MASTER_NACK : I2C_MASTER_ACK);
        }
        i2c_master_stop(cmd);
        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return err;
}

/*--------------------------------------
   Задача: непрерывный мониторинг и 30-секундная запись при тряске
--------------------------------------*/
static void read_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(200));  // даем MPU проснуться

    // Пробуждаем MPU6050
    mpu6050_write_reg(MPU6050_PWR_MGMT_1_REG, 0x00);

    bool recording = false;
    FILE *f = NULL;
    uint64_t start_time = 0;

    while (1) {
        uint8_t raw[14];
        esp_err_t err = mpu6050_read_all(raw);
        if (err != ESP_OK) {
            // Если чтение не удалось, попытаться пробудить MPU и подождать
            mpu6050_write_reg(MPU6050_PWR_MGMT_1_REG, 0x00);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int16_t raw_ax = (raw[0]  << 8) | raw[1];
        int16_t raw_ay = (raw[2]  << 8) | raw[3];
        int16_t raw_az = (raw[4]  << 8) | raw[5];
        int16_t raw_gx = (raw[8]  << 8) | raw[9];
        int16_t raw_gy = (raw[10] << 8) | raw[11];
        int16_t raw_gz = (raw[12] << 8) | raw[13];

        float ax = raw_ax / ACCEL_SCALE_FACTOR;
        float ay = raw_ay / ACCEL_SCALE_FACTOR;
        float az = raw_az / ACCEL_SCALE_FACTOR;
        float gx = raw_gx / GYRO_SCALE_FACTOR;
        float gy = raw_gy / GYRO_SCALE_FACTOR;
        float gz = raw_gz / GYRO_SCALE_FACTOR;

        float accel_total = sqrtf(ax*ax + ay*ay + az*az);

        // Если еще не записываем и встряска > порога, начинаем новый цикл записи
        if (!recording && accel_total > 2.5f) {
            recording = true;
            start_time = esp_timer_get_time();

            // Открываем /spiffs/accel.txt (перезаписываем)
            f = fopen("/spiffs/accel.txt", "w");
            if (!f) {
                int errn = errno;
                ESP_LOGE(TAG_SPI,
                         "Не удалось открыть /spiffs/accel.txt (errno=%d %s)",
                         errn, strerror(errn));
                recording = false;
            } else {
                fprintf(f, "{\"start_time\": %" PRIu64 "}", start_time);
                fflush(f);
            }
        }

        // Если в режиме записи, продолжаем писать, пока не пройдут 30 секунд
        if (recording) {
            uint64_t now = esp_timer_get_time();
            uint64_t elapsed = now - start_time;
            if (elapsed <= 30000000ULL) {
                fprintf(f,
                        " {\"t\": %" PRIu64 ", \"ax\": %.3f, \"ay\": %.3f, \"az\": %.3f, \"gx\": %.2f, \"gy\": %.2f, \"gz\": %.2f}",
                        elapsed, ax, ay, az, gx, gy, gz);
                fflush(f);
            } else {
                // 30 секунд истекли, закрываем файл и возвращаемся к ожиданию следующей встряски
                fclose(f);
                recording = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // читаем с шагом 100 мс
    }
}

/*--------------------------------------
   HTTP GET /log handler
--------------------------------------*/
static esp_err_t log_get_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/accel.txt", "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Файл не найден");
        return ESP_FAIL;
    }
    char buf[256];
    while (fgets(buf, sizeof(buf), f)) {
        httpd_resp_send_chunk(req, buf, strlen(buf));
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t log_uri = {
    .uri       = "/log",
    .method    = HTTP_GET,
    .handler   = log_get_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &log_uri);
        ESP_LOGI(TAG_HTTP, "HTTP-сервер запущен, переходите на http://192.168.4.1/log");
    } else {
        ESP_LOGE(TAG_HTTP, "Не удалось запустить HTTP-сервер");
    }
    return server;
}

/*--------------------------------------
   app_main: инициализация всего
--------------------------------------*/
void app_main(void)
{
    esp_err_t ret = init_spiffs();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SPI, "SPIFFS смонтирован, записываем test.txt");
        FILE *f = fopen("/spiffs/test.txt", "w");
        if (f) {
            fprintf(f, "Hello, SPIFFS!\n");
            fclose(f);
            ESP_LOGI(TAG_SPI, "Успех — test.txt создан");
        } else {
            int e = errno;
            ESP_LOGE(TAG_SPI,
                     "Не удалось открыть /spiffs/test.txt (errno=%d %s)",
                     e, strerror(e));
        }
    }

    wifi_init_softap();
    start_webserver();

    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(read_task, "read_task", 4096, NULL, 5, NULL);
}
