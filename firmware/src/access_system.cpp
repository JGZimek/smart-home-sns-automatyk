#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>

#include "module_config.h"
#include "access_system.h"
#include "mqtt_core.h"
#include "rc522.h"

#if defined(MODULE_ACCESS)

static const char* TAG_ACC = "ACCESS_SYS";

// --- KONFIGURACJA PINÓW ---
// Zamek (Serwomechanizm)
#define SERVO_PIN GPIO_NUM_4

// SPI & RFID
#define PIN_RFID_MISO GPIO_NUM_19
#define PIN_RFID_MOSI GPIO_NUM_23
#define PIN_RFID_SCK  GPIO_NUM_18
#define PIN_RFID_SDA  GPIO_NUM_5

// Klawiatura 4x4
const gpio_num_t ROW_PINS[4] = {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26};
const gpio_num_t COL_PINS[4] = {GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13};

const char KEY_MAP[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static rc522_handle_t rfid_scanner;

// =========================================================================
// NATYWNA OBSŁUGA SERWA (LEDC)
// =========================================================================
void init_servo() {
    // Konfiguracja timera dla 50Hz (standard serw modelarskich)
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_13_BIT; // Rozdzielczość 0-8191
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = SERVO_PIN;
    ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}

// Funkcja ustawiająca kąt serwa (0 - 180 stopni)
void set_servo_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // Przy 50Hz pełny cykl to 20ms
    // Min impuls ~0.5ms (duty=204), Max impuls ~2.5ms (duty=1024)
    int duty = 204 + ((1024 - 204) * angle) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// =========================================================================
// OBSŁUGA RFID (Sprzętowy Callback)
// =========================================================================
static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;
    
    if (event_id == RC522_EVENT_TAG_SCANNED) {
        rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
        ESP_LOGI(TAG_ACC, "RFID Tag Scanned: %llu", tag->serial_number);
        
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"uid\": \"%llu\"}", tag->serial_number);
        mqtt_publish("home/access/rfid", payload, 1, 0);
    }
}

// =========================================================================
// ZADANIE: KLAWIATURA 4x4
// =========================================================================
static void keypad_task(void *pvParameters) {
    for(int i = 0; i < 4; i++) {
        gpio_set_direction(ROW_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(ROW_PINS[i], 1); 
        
        gpio_set_direction(COL_PINS[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(COL_PINS[i], GPIO_PULLUP_ONLY);
    }

    char last_key = 0;

    while(1) {
        char pressed_key = 0;
        
        for(int r = 0; r < 4; r++) {
            gpio_set_level(ROW_PINS[r], 0); // Aktywacja wiersza do masy
            vTaskDelay(pdMS_TO_TICKS(5));   // Krótkie opóźnienie na ustabilizowanie
            
            for(int c = 0; c < 4; c++) {
                if(gpio_get_level(COL_PINS[c]) == 0) {
                    pressed_key = KEY_MAP[r][c];
                    break;
                }
            }
            gpio_set_level(ROW_PINS[r], 1); // Odłączenie wiersza
            if (pressed_key != 0) break;
        }

        // Debouncing i publikacja
        if (pressed_key != 0 && pressed_key != last_key) {
            ESP_LOGI(TAG_ACC, "Key Pressed: %c", pressed_key);
            char payload[32];
            snprintf(payload, sizeof(payload), "{\"key\": \"%c\"}", pressed_key);
            mqtt_publish("home/access/keypad", payload, 1, 0);
        }
        
        last_key = pressed_key;
        vTaskDelay(pdMS_TO_TICKS(50)); // Odpytywanie z częstotliwością 20Hz
    }
}

// =========================================================================
// OBSŁUGA MQTT (Rozkaz otwarcia drzwi)
// =========================================================================
void access_mqtt_callback(const char* topic, const char* data, int data_len) {
    if (strncmp(topic, "home/access/door/set", strlen("home/access/door/set")) == 0) {
        if (strncmp(data, "OPEN", 4) == 0) {
            ESP_LOGI(TAG_ACC, "Otwieram zamek na 5 sekund!");
            set_servo_angle(90); // Otwarcie
            mqtt_publish("home/access/door/state", "OPEN", 1, 1);
            
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            ESP_LOGI(TAG_ACC, "Zamykam zamek.");
            set_servo_angle(0);  // Zamknięcie
            mqtt_publish("home/access/door/state", "CLOSED", 1, 1);
        }
    }
}

// =========================================================================
// INICJALIZACJA SYSTEMU
// =========================================================================
void init_access_system(void) {
    ESP_LOGI(TAG_ACC, "Initializing Access Hardware...");

    // 1. Zamek (Servo)
    init_servo();
    set_servo_angle(0); // Domyślnie zaryglowane

    // 2. Czytnik RFID (RC522)
    rc522_config_t config = {};
    config.spi.host = SPI3_HOST; // Nowy standard w ESP-IDF (zamiast VSPI_HOST)
    config.spi.miso_gpio = PIN_RFID_MISO;
    config.spi.mosi_gpio = PIN_RFID_MOSI;
    config.spi.sck_gpio  = PIN_RFID_SCK;
    config.spi.sda_gpio  = PIN_RFID_SDA;

    rc522_create(&config, &rfid_scanner);
    rc522_register_events(rfid_scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    rc522_start(rfid_scanner);

    // 3. Klawiatura 
    xTaskCreatePinnedToCore(keypad_task, "keypad_task", 3072, NULL, 5, NULL, 1);

    ESP_LOGI(TAG_ACC, "Access System Initialized!");
}

#endif // MODULE_ACCESS