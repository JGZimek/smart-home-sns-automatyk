#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <string.h>

#include "module_config.h"
#include "security_system.h"
#include "mqtt_core.h"

#if defined(MODULE_SECURITY)

static const char* TAG_SEC = "SECURITY";

// --- KONFIGURACJA PINÓW ---
#define PIN_PIR_1    GPIO_NUM_14
#define PIN_PIR_2    GPIO_NUM_27
#define PIN_FLAME    GPIO_NUM_26
#define PIN_GAS      GPIO_NUM_34 // ADC1_CH6
#define PIN_SIREN    GPIO_NUM_25

#define GAS_THRESHOLD 2000

static volatile bool systemArmed = false;
static adc_oneshot_unit_handle_t adc1_handle;

// --- OBSŁUGA MQTT (Odbiór) ---
void security_mqtt_callback(const char* topic, const char* data, int data_len) {
    if (strncmp(topic, "home/security/arm/set", strlen("home/security/arm/set")) == 0) {
        if (strncmp(data, "ARM", 3) == 0) {
            systemArmed = true;
            ESP_LOGW(TAG_SEC, "SYSTEM ARMED!");
            mqtt_publish("home/security/status", "{\"val\": 1}", 1, 1);
        } else if (strncmp(data, "DISARM", 6) == 0) {
            systemArmed = false;
            gpio_set_level(PIN_SIREN, 0); // Wyłącz syrenę przy rozbrojeniu
            ESP_LOGW(TAG_SEC, "SYSTEM DISARMED");
            // mqtt_publish("home/security/status", "{\"val\": 0}", 1, 1);
        }
    }
}

// --- GŁÓWNE ZADANIE SYSTEMU ---
static void security_task(void *pvParameters) {
    ESP_LOGI(TAG_SEC, "Security Task Started");

    while (1) {
        // 1. ODCZYT PIR (Tylko jeśli uzbrojony)
        if (systemArmed) {
            if (gpio_get_level(PIN_PIR_1) == 1) {
                ESP_LOGW(TAG_SEC, "Motion Detected PIR 1");
                gpio_set_level(PIN_SIREN, 1);
                // mqtt_publish("home/security/motion/1", "{\"val\": 1}", 1, 0);
                vTaskDelay(pdMS_TO_TICKS(5000));
                gpio_set_level(PIN_SIREN, 0);
            }
            if (gpio_get_level(PIN_PIR_2) == 1) {
                ESP_LOGW(TAG_SEC, "Motion Detected PIR 2");
                gpio_set_level(PIN_SIREN, 1);
                // mqtt_publish("home/security/motion/2", "{\"val\": 1}", 1, 0);
                vTaskDelay(pdMS_TO_TICKS(5000));
                gpio_set_level(PIN_SIREN, 0);
            }
        }

        // 2. ODCZYT OGNIOWY (Zawsze aktywny)
        if (gpio_get_level(PIN_FLAME) == 0) { // Często moduły dają stan niski przy płomieniu
            ESP_LOGE(TAG_SEC, "FIRE DETECTED!");
            gpio_set_level(PIN_SIREN, 1);
            // mqtt_publish("home/security/fire", "{\"val\": 1}", 1, 0);
        }

        // 3. ODCZYT GAZU (Zawsze aktywny)
        int gasValue = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &gasValue));
        if (gasValue > GAS_THRESHOLD) {
            ESP_LOGE(TAG_SEC, "GAS LEAK! Level: %d", gasValue);
            gpio_set_level(PIN_SIREN, 1);
            
            // char payload[32];
            // snprintf(payload, sizeof(payload), "{\"val\": %d}", gasValue);
            // mqtt_publish("home/security/gas", payload, 1, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Pętla działa z częstotliwością 5Hz
    }
}

// --- INICJALIZACJA ---
void init_security_system(void) {
    ESP_LOGI(TAG_SEC, "Initializing Security Hardware...");

    // Konfiguracja pinów wejściowych (PIR, Flame)
    gpio_config_t io_in_conf = {};
    io_in_conf.intr_type = GPIO_INTR_DISABLE;
    io_in_conf.mode = GPIO_MODE_INPUT;
    io_in_conf.pin_bit_mask = (1ULL << PIN_PIR_1) | (1ULL << PIN_PIR_2) | (1ULL << PIN_FLAME);
    io_in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_in_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_in_conf));

    // Konfiguracja pinu wyjściowego (Syrena)
    gpio_config_t io_out_conf = {};
    io_out_conf.intr_type = GPIO_INTR_DISABLE;
    io_out_conf.mode = GPIO_MODE_OUTPUT;
    io_out_conf.pin_bit_mask = (1ULL << PIN_SIREN);
    io_out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_out_conf));
    gpio_set_level(PIN_SIREN, 0);

    // Konfiguracja ADC dla czujnika gazu
    adc_oneshot_unit_init_cfg_t init_config1 = {};
    init_config1.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12; // Tłumienie 11/12dB pozwala na pomiar 0-~3.3V
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));

    // Uruchomienie zadania FreeRTOS
    xTaskCreatePinnedToCore(security_task, "sec_task", 4096, NULL, 5, NULL, 1);
    ESP_LOGI(TAG_SEC, "Security System Initialized!");
}

#endif // MODULE_SECURITY