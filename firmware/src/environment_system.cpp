#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <time.h>

#include "module_config.h"
#include "environment_system.h"
#include "mqtt_core.h"

#if defined(MODULE_ENV)

// Prawidłowa biblioteka ze zbioru esp-idf-lib
#include <bmp280.h>
#include <i2cdev.h>

static const char* TAG_ENV = "ENV_SYS";

// --- KONFIGURACJA PINÓW I I2C ---
#define PIN_FAN_COOLING GPIO_NUM_18
#define PIN_FAN_VENT    GPIO_NUM_19

#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define INA1_ADDR       0x45 // Watomierz 1 (Solar)
#define INA2_ADDR       0x44 // Watomierz 2 (Battery)
#define TSL2591_ADDR    0x29

// --- STRUKTURY DANYCH ---
struct FanCommand {
    uint8_t fanId; 
    bool state;    
};

static QueueHandle_t cmdQueue;

// Deskryptory i2cdev dla czujników obsługiwanych "ręcznie" (INA219 x2, TSL2591).
// BME280 ma własny deskryptor zarządzany przez bibliotekę bmp280. Wszystkie
// współdzielą jeden port I2C zarządzany przez i2cdev (NOWY sterownik) — dzięki
// czemu nie ma konfliktu starego i nowego sterownika I2C.
static i2c_dev_t ina1_dev;  // Solar
static i2c_dev_t ina2_dev;  // Battery
static i2c_dev_t tsl_dev;   // TSL2591

static void i2cdev_descriptor_init(i2c_dev_t *d, uint8_t addr) {
    memset(d, 0, sizeof(*d));
    d->port = I2C_MASTER_NUM;
    d->addr = addr;
    d->cfg.sda_io_num = I2C_MASTER_SDA_IO;
    d->cfg.scl_io_num = I2C_MASTER_SCL_IO;
    d->cfg.sda_pullup_en = 1;
    d->cfg.scl_pullup_en = 1;
    d->cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_dev_create_mutex(d);
}

// =========================================================================
// ODCZYTY I2C przez i2cdev (INA219, TSL2591)
// =========================================================================

static bool read_ina219(i2c_dev_t *dev, float *voltage, float *current, float *power) {
    uint8_t buf[2];
    if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    esp_err_t e_bus   = i2c_dev_read_reg(dev, 0x02, buf, 2);     // rejestr bus voltage
    uint16_t bus_val  = (buf[0] << 8) | buf[1];
    esp_err_t e_shunt = i2c_dev_read_reg(dev, 0x01, buf, 2);     // rejestr shunt voltage
    int16_t shunt_val = (buf[0] << 8) | buf[1];
    xSemaphoreGive(dev->mutex);
    if (e_bus != ESP_OK || e_shunt != ESP_OK) return false;

    *voltage = (bus_val >> 3) * 0.004f;
    float shunt_mV = shunt_val * 0.01f;
    *current = shunt_mV / 0.1f;
    *power = (*voltage) * (*current);
    return true;
}

static bool init_tsl2591(i2c_dev_t *dev) {
    uint8_t enable = 0x03; // ENABLE: PON | AEN
    if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    esp_err_t e = i2c_dev_write_reg(dev, 0xA0 | 0x00, &enable, 1);
    xSemaphoreGive(dev->mutex);
    return e == ESP_OK;
}

static bool read_tsl2591(i2c_dev_t *dev, float *lux) {
    uint8_t buf[2];
    if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    esp_err_t e_ch0 = i2c_dev_read_reg(dev, 0xA0 | 0x14, buf, 2);
    uint16_t ch0 = (buf[1] << 8) | buf[0];
    esp_err_t e_ch1 = i2c_dev_read_reg(dev, 0xA0 | 0x16, buf, 2);
    uint16_t ch1 = (buf[1] << 8) | buf[0];
    xSemaphoreGive(dev->mutex);
    if (e_ch0 != ESP_OK || e_ch1 != ESP_OK) return false;

    if (ch0 == 0) { *lux = 0; return true; }
    float cpl = (100.0F * 25.0F) / 408.0F;
    float calc_lux = ((float)ch0 - (float)ch1) * (1.0F - ((float)ch1 / (float)ch0)) / cpl;
    *lux = calc_lux > 0 ? calc_lux : 0;
    return true;
}

// =========================================================================

// --- OBSŁUGA MQTT (Odbiór sterowania) ---
void environment_mqtt_callback(const char* topic, const char* data, int data_len) {
    FanCommand cmd = {0, false};
    bool validCommand = false;

    if (strstr(topic, "cooling/set")) {
        cmd.fanId = 1;
        validCommand = true;
    } else if (strstr(topic, "vent/set")) {
        cmd.fanId = 2;
        validCommand = true;
    }

    if (validCommand) {
        if (strncmp(data, "ON", 2) == 0) cmd.state = true;
        else if (strncmp(data, "OFF", 3) == 0) cmd.state = false;
        else validCommand = false;
    }

    if (validCommand) {
        if (xQueueSend(cmdQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_ENV, "Kolejka komend pełna!");
        }
    }
}

// --- ZADANIE: STEROWANIE WENTYLATORAMI ---
static void control_task(void *pvParameters) {
    ESP_LOGI(TAG_ENV, "Control Task Started");
    FanCommand cmd;

    while (1) {
        if (xQueueReceive(cmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            gpio_num_t pin = (cmd.fanId == 1) ? PIN_FAN_COOLING : PIN_FAN_VENT;
            int level = cmd.state ? 0 : 1; 
            gpio_set_level(pin, level);
            
            ESP_LOGI(TAG_ENV, "Fan %d ustawiony na: %s", cmd.fanId, cmd.state ? "ON" : "OFF");

            const char* topic = (cmd.fanId == 1) ? "home/garden/fan/cooling/state" : "home/garden/fan/vent/state";
            mqtt_publish(topic, cmd.state ? "ON" : "OFF", 1, 1);
        }
    }
}

// --- ZADANIE: ODCZYT CZUJNIKÓW I PUBLIKACJA ---
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG_ENV, "Sensor Task Started");

    // 0. Inicjalizacja podsystemu i2cdev (raz). Zarządza on jedną magistralą I2C
    //    na porcie, współdzieloną przez wszystkie czujniki — bez konfliktu sterowników.
    if (i2cdev_init() != ESP_OK) {
        ESP_LOGE(TAG_ENV, "i2cdev_init nieudane — odczyty I2C niedostepne");
    }

    // 1. BME280 (biblioteka bmp280 na i2cdev)
    bmp280_t bme_dev;
    memset(&bme_dev, 0, sizeof(bmp280_t));
    bool bme_ok = false;
    if (bmp280_init_desc(&bme_dev, BMP280_I2C_ADDRESS_0, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) == ESP_OK) {
        bme_dev.i2c_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ; // 100 kHz jak reszta magistrali (domyślnie lib daje 1 MHz)
        bmp280_params_t params;
        bmp280_init_default_params(&params);
        if (bmp280_init(&bme_dev, &params) == ESP_OK) {
            bme_ok = true;
            bool is_bme280 = bme_dev.id == BME280_CHIP_ID;
            ESP_LOGI(TAG_ENV, "Czujnik %s odnaleziony i skalibrowany", is_bme280 ? "BME280" : "BMP280");
        }
    }
    if (!bme_ok) ESP_LOGE(TAG_ENV, "Nie odnaleziono BME280/BMP280");

    // 2. Deskryptory INA219 x2 + TSL2591 (ten sam port I2C przez i2cdev)
    i2cdev_descriptor_init(&ina1_dev, INA1_ADDR);
    i2cdev_descriptor_init(&ina2_dev, INA2_ADDR);
    i2cdev_descriptor_init(&tsl_dev, TSL2591_ADDR);

    bool tsl_ok = init_tsl2591(&tsl_dev);
    if (tsl_ok) ESP_LOGI(TAG_ENV, "TSL2591 odnaleziony i zainicjowany");
    else ESP_LOGE(TAG_ENV, "Nie odnaleziono TSL2591");

    char payload[128];

    while (1) {
        time_t now;
        time(&now);

        // Odczyt Światła
        float lux;
        if (tsl_ok && read_tsl2591(&tsl_dev, &lux)) {
            snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", lux, (long long)now);
            mqtt_publish("home/garden/environment/light", payload, 1, 0);
        }

        // Odczyt Atmosfery
        if (bme_ok) {
            float temp, pres, hum;
            if (bmp280_read_float(&bme_dev, &temp, &pres, &hum) == ESP_OK) {
                snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", temp, (long long)now);
                mqtt_publish("home/garden/environment/temperature", payload, 1, 0);

                if (bme_dev.id == BME280_CHIP_ID) {
                    snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", hum, (long long)now);
                    mqtt_publish("home/garden/environment/humidity", payload, 1, 0);
                }

                snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", pres / 100.0, (long long)now);
                mqtt_publish("home/garden/environment/pressure", payload, 1, 0);
            }
        }

        // Odczyt Solara
        float v, c, p;
        if (read_ina219(&ina1_dev, &v, &c, &p)) {
            snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", v, (long long)now);
            mqtt_publish("home/garden/power/solar/voltage", payload, 1, 0);
            snprintf(payload, sizeof(payload), "{\"value\": %.4f, \"ts\": %lld}", c / 1000.0, (long long)now);
            mqtt_publish("home/garden/power/solar/current", payload, 1, 0);
            snprintf(payload, sizeof(payload), "{\"value\": %.4f, \"ts\": %lld}", p / 1000.0, (long long)now);
            mqtt_publish("home/garden/power/solar/power", payload, 1, 0);
        }

        // Odczyt Baterii
        if (read_ina219(&ina2_dev, &v, &c, &p)) {
            snprintf(payload, sizeof(payload), "{\"value\": %.2f, \"ts\": %lld}", v, (long long)now);
            mqtt_publish("home/garden/power/battery/voltage", payload, 1, 0);
            snprintf(payload, sizeof(payload), "{\"value\": %.4f, \"ts\": %lld}", c / 1000.0, (long long)now);
            mqtt_publish("home/garden/power/battery/current", payload, 1, 0);
            snprintf(payload, sizeof(payload), "{\"value\": %.4f, \"ts\": %lld}", p / 1000.0, (long long)now);
            mqtt_publish("home/garden/power/battery/power", payload, 1, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- GŁÓWNA INICJALIZACJA SYSTEMU ---
void init_environment_system(void) {
    ESP_LOGI(TAG_ENV, "Initializing Environment Hardware...");

    // 1. Konfiguracja przekaźników
    gpio_config_t io_out_conf = {};
    io_out_conf.intr_type = GPIO_INTR_DISABLE;
    io_out_conf.mode = GPIO_MODE_OUTPUT;
    io_out_conf.pin_bit_mask = (1ULL << PIN_FAN_COOLING) | (1ULL << PIN_FAN_VENT);
    io_out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_out_conf));

    gpio_set_level(PIN_FAN_COOLING, 1);
    gpio_set_level(PIN_FAN_VENT, 1);

    // 2. Magistralę I2C stawia sensor_task przez i2cdev (jeden właściciel portu,
    //    bez konfliktu starego i nowego sterownika I2C).

    // 3. Utworzenie kolejki sterującej
    cmdQueue = xQueueCreate(10, sizeof(FanCommand));

    // 4. Delegacja pracy do dedykowanych zadań sprzętowych FreeRTOS
    xTaskCreatePinnedToCore(control_task, "ctrl_task", 3072, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensor_task, "sens_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG_ENV, "Environment System Initialized!");
}

#endif // MODULE_ENV