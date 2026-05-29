#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>

#include "module_config.h"
#include "environment_system.h"
#include "mqtt_core.h"

#if defined(MODULE_ENV)

static const char* TAG_ENV = "ENV_SYS";

// --- KONFIGURACJA PINÓW I I2C ---
#define PIN_FAN_COOLING GPIO_NUM_18
#define PIN_FAN_VENT    GPIO_NUM_19

#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

// --- STRUKTURY DANYCH ---
struct FanCommand {
    uint8_t fanId; // 1 = Cooling, 2 = Ventilation
    bool state;    // true=ON, false=OFF
};

// Kolejki
static QueueHandle_t cmdQueue; 

// --- OBSŁUGA MQTT (Odbiór) ---
void environment_mqtt_callback(const char* topic, const char* data, int data_len) {
    FanCommand cmd = {0, false};
    bool validCommand = false;

    if (strncmp(topic, "home/garden/fan/cooling/set", strlen("home/garden/fan/cooling/set")) == 0) {
        cmd.fanId = 1;
        validCommand = true;
    } else if (strncmp(topic, "home/garden/fan/vent/set", strlen("home/garden/fan/vent/set")) == 0) {
        cmd.fanId = 2;
        validCommand = true;
    }

    if (validCommand) {
        if (strncmp(data, "ON", 2) == 0) cmd.state = true;
        else if (strncmp(data, "OFF", 3) == 0) cmd.state = false;
        else validCommand = false;
    }

    if (validCommand) {
        // Wysyłamy komendę do zadania sterującego
        if (xQueueSend(cmdQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_ENV, "Kolejka komend pelna!");
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
            
            // Logika odwrócona dla przekaźników (Active LOW)
            int level = cmd.state ? 0 : 1;
            gpio_set_level(pin, level);
            
            ESP_LOGI(TAG_ENV, "Fan %d ustawiony na: %s", cmd.fanId, cmd.state ? "ON" : "OFF");

            // Feedback MQTT
            const char* topic = (cmd.fanId == 1) ? "home/garden/fan/cooling/state" : "home/garden/fan/vent/state";
            mqtt_publish(topic, cmd.state ? "ON" : "OFF", 1, 1);
        }
    }
}

// --- ZADANIE: ODCZYT CZUJNIKÓW I2C ---
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG_ENV, "Sensor Task Started");

    while (1) {
        // TUTAJ ZNAJDZIE SIĘ LOGIKA ODCZYTU CZUJNIKÓW
        // Wymaga dodania bibliotek BME280/INA219 pod ESP-IDF
        // ...
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- INICJALIZACJA ---
void init_environment_system(void) {
    ESP_LOGI(TAG_ENV, "Initializing Environment Hardware...");

    // 1. Konfiguracja wyjść (Wentylatory)
    gpio_config_t io_out_conf = {};
    io_out_conf.intr_type = GPIO_INTR_DISABLE;
    io_out_conf.mode = GPIO_MODE_OUTPUT;
    io_out_conf.pin_bit_mask = (1ULL << PIN_FAN_COOLING) | (1ULL << PIN_FAN_VENT);
    io_out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_out_conf));

    // Stan początkowy: Wyłączone (Wysoki stan dla modułu przekaźnika)
    gpio_set_level(PIN_FAN_COOLING, 1);
    gpio_set_level(PIN_FAN_VENT, 1);

    // 2. Inicjalizacja sterownika I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; // Standardowe taktowanie
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    // 3. Utworzenie Kolejek
    cmdQueue = xQueueCreate(10, sizeof(FanCommand));

    // 4. Uruchomienie Zadań
    xTaskCreatePinnedToCore(control_task, "ctrl_task", 3072, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensor_task, "sens_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG_ENV, "Environment System Initialized!");
}

#endif // MODULE_ENV