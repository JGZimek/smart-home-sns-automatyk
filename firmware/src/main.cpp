#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h" // Wymagane przez ESP-IDF do działania WiFi

static const char *TAG = "MAIN";

// ==========================================================
// 1. SYSTEM BEZPIECZEŃSTWA (SECURITY)
// ==========================================================
#ifdef MODULE_SECURITY
void setupModule() {
    ESP_LOGI(TAG, "Inicjalizacja modulu: SECURITY");
}

// ==========================================================
// 2. SYSTEM KONTROLI DOSTĘPU (ACCESS)
// ==========================================================
#elif defined(MODULE_ACCESS)
void setupModule() {
    ESP_LOGI(TAG, "Inicjalizacja modulu: ACCESS");
}

// ==========================================================
// 3. SYSTEM ŚRODOWISKOWY (ENV)
// ==========================================================
#elif defined(MODULE_ENV)
void setupModule() {
    ESP_LOGI(TAG, "Inicjalizacja modulu: ENVIRONMENT");
}

// ==========================================================
// 4. WERSJA PODSTAWOWA (DEFAULT)
// ==========================================================
#else
void setupModule() {
    ESP_LOGI(TAG, "Inicjalizacja modulu: DEFAULT (Czysty start)");
}
#endif

// ==========================================================
// GŁÓWNA FUNKCJA (WSPÓLNA DLA WSZYSTKICH)
// ==========================================================
extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Start SmartHome Node (ESP-IDF) ===");

    // 1. Inicjalizacja NVS (Fundament systemu ESP32)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Jeśli struktura pamięci jest uszkodzona/nowa, wyczyść i spróbuj ponownie
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Piec NVS zainicjowany poprawnie.");

    // 2. Setup konkretnego modułu
    setupModule();

    // 3. Pętla główna (w ESP-IDF to też jest Task, utrzymujemy go przy życiu)
    while (1) {
        // Co 10 sekund wypisz status do konsoli
        ESP_LOGI(TAG, "System dziala w tle...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}