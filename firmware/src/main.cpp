#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "module_config.h"
#include "network_core.h"
#include "mqtt_core.h"
#include "device_core.h"

// Dodajemy nagłówki zależne od wybranego środowiska
#if defined(MODULE_SECURITY)
    #include "security_system.h"
#elif defined(MODULE_ENV)
    #include "environment_system.h"
#elif defined(MODULE_ACCESS)
    #include "access_system.h"
#endif

static const char *TAG = "MAIN";

void setupModule() { 
    ESP_LOGW(TAG, "=============================================");
    ESP_LOGW(TAG, "Wersja v2.0: Moduł %s ", MODULE_NAME);
    ESP_LOGW(TAG, "=============================================");
}

// Główna funkcja wymagan przez ESP-IDF
extern "C" void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setupModule();
    
    // Inicjalizacja sieci (WiFi Provisioning), która z kolei uruchomi MQTT
    wifi_init_and_prov();

    // Wspólny heartbeat diagnostyczny (publikuje się dopiero po połączeniu MQTT)
    device_start_diag_task();

    // Start sprzętu i logiki dla wybranej platformy
#if defined(MODULE_SECURITY)
    init_security_system();
#elif defined(MODULE_ENV)
    init_environment_system();
#elif defined(MODULE_ACCESS)
    init_access_system();
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}