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
#define SIREN_HOLD_MS 5000   // jak długo syrena gra po ustąpieniu zdarzenia

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
            mqtt_publish("home/security/status", "{\"val\": 0}", 1, 1);
        }
    }
}

// --- GŁÓWNE ZADANIE SYSTEMU ---
// Pętla 5 Hz, w pełni nieblokująca. Zdarzenia publikowane są na zboczu
// (zmiana stanu), aby nie zaśmiecać brokera. Syrena jest sterowana
// jednym punktem na końcu pętli i przytrzymywana przez SIREN_HOLD_MS,
// dzięki czemu wykrycie ognia/gazu nie jest „zagłuszane” obsługą PIR.
static void security_task(void *pvParameters) {
    ESP_LOGI(TAG_SEC, "Security Task Started");

    bool prev_pir1 = false, prev_pir2 = false, prev_fire = false, prev_gas = false;
    TickType_t siren_off_at = 0;

    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool active = false; // czy w tej iteracji występuje aktywne zagrożenie

        // 1. PIR – brany pod uwagę tylko w trybie uzbrojonym
        bool pir1 = systemArmed && gpio_get_level(PIN_PIR_1) == 1;
        bool pir2 = systemArmed && gpio_get_level(PIN_PIR_2) == 1;
        if (pir1 && !prev_pir1) {
            ESP_LOGW(TAG_SEC, "Motion Detected PIR 1");
            mqtt_publish("home/security/motion/1", "{\"val\": 1}", 1, 0);
        }
        if (pir2 && !prev_pir2) {
            ESP_LOGW(TAG_SEC, "Motion Detected PIR 2");
            mqtt_publish("home/security/motion/2", "{\"val\": 1}", 1, 0);
        }
        prev_pir1 = pir1;
        prev_pir2 = pir2;
        if (pir1 || pir2) active = true;

        // 2. Czujnik płomienia – zawsze aktywny (stan niski = płomień)
        bool fire = gpio_get_level(PIN_FLAME) == 0;
        if (fire != prev_fire) {
            // Uwaga: format ESP_LOGx musi być literałem (makro skleja go z sąsiednimi stringami).
            if (fire) ESP_LOGE(TAG_SEC, "FIRE DETECTED!");
            else      ESP_LOGI(TAG_SEC, "Fire cleared");
            mqtt_publish("home/security/fire", fire ? "{\"val\": 1}" : "{\"val\": 0}", 1, 1);
            prev_fire = fire;
        }
        if (fire) active = true;

        // 3. Czujnik gazu (ADC) – zawsze aktywny; błąd odczytu nie restartuje węzła
        int gasValue = 0;
        if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &gasValue) == ESP_OK) {
            bool gas = gasValue > GAS_THRESHOLD;
            if (gas != prev_gas) {
                if (gas) ESP_LOGE(TAG_SEC, "GAS LEAK! Level: %d", gasValue);
                else     ESP_LOGI(TAG_SEC, "Gas cleared (%d)", gasValue);
                char payload[32];
                snprintf(payload, sizeof(payload), "{\"val\": %d}", gasValue);
                mqtt_publish("home/security/gas", payload, 1, 1);
                prev_gas = gas;
            }
            if (gas) active = true;
        } else {
            ESP_LOGW(TAG_SEC, "Blad odczytu ADC czujnika gazu");
        }

        // 4. Sterowanie syreną – jeden punkt decyzyjny, z przytrzymaniem
        if (active) {
            siren_off_at = now + pdMS_TO_TICKS(SIREN_HOLD_MS);
        }
        gpio_set_level(PIN_SIREN, (now < siren_off_at) ? 1 : 0);

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