#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "module_config.h"
#include "network_core.h"

static const char *TAG = "MAIN";

void setupModule() { 
    ESP_LOGW(TAG, "=============================================");
    ESP_LOGW(TAG, "Wersja v2.0: Moduł %s ", MODULE_NAME);
    ESP_LOGW(TAG, "=============================================");
}

extern "C" void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setupModule();
    
    wifi_init_and_prov();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}