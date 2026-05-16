#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// Wspólny tag do logów
static const char *TAG = "MAIN";

// ==========================================
// MIEJSCE NA FUNKCJE SPECYFICZNE DLA MODUŁÓW
// ==========================================

#ifdef MODULE_ACCESS
    void setup_module() {
        ESP_LOGI(TAG, "Inicjalizacja: SYSTEM DOSTEPU (Access)");
        // Tutaj w przyszłości damy logikę RFID, Serwo, Klawiatury
    }

#elif defined(MODULE_ENV)
    void setup_module() {
        ESP_LOGI(TAG, "Inicjalizacja: SYSTEM SRODOWISKOWY (Environment)");
        // Tutaj w przyszłości damy I2C, BME280, INA219
    }

#elif defined(MODULE_SECURITY)
    void setup_module() {
        ESP_LOGI(TAG, "Inicjalizacja: SYSTEM ALARMOWY (Security)");
        // Tutaj w przyszłości obsługa PIR, Gazu, Syreny
    }

#else
    void setup_module() {
        ESP_LOGI(TAG, "Inicjalizacja: WERSJA PODSTAWOWA (Default)");
        // Pusta wersja do testowania ESP32
    }
#endif

// ==========================================
// GŁÓWNA FUNKCJA (WSPÓLNA)
// ==========================================
extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Start SmartHome Node ===");

    // Tutaj w przyszłości dodamy wspólną inicjalizację:
    // 1. NVS (Pamięć Flash)
    // 2. WiFi
    // 3. Klient MQTT

    // Uruchomienie logiki specyficznej dla wybranego środowiska
    setup_module();

    // W ESP-IDF nie ma funkcji loop(). Zadania chodzą w tle (FreeRTOS).
    // Możemy uśpić ten główny wątek, albo zrobić w nim prostą pętlę idle.
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}