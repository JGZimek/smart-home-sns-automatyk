#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"

// --- Wymagane przez WiFi Provisioning ---
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

static const char *TAG = "MAIN";
static const char *TAG_PROV = "PROV";

// ==========================================================
// 1. OBSŁUGA ZDARZEŃ (WIFI I PROVISIONING)
// ==========================================================
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG_PROV, "===============================================");
                ESP_LOGI(TAG_PROV, " PROVISIONING AKTYWNY (BLE)!");
                ESP_LOGI(TAG_PROV, " Uzyj aplikacji 'ESP BLE Prov' (iOS/Android)");
                ESP_LOGI(TAG_PROV, " Nazwa urzadzenia BLE: PROV_SmartHome");
                ESP_LOGI(TAG_PROV, "===============================================");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG_PROV, "Otrzymano SSID: %s", (const char *) wifi_sta_cfg->ssid);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG_PROV, "Blad autoryzacji WiFi (Zle haslo)! Kod bledu: %d\n", *reason);
                wifi_prov_mgr_reset_sm_state_on_failure();
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG_PROV, "Autoryzacja udana! ESP32 laczy sie z domowym WiFi...");
                break;
            case WIFI_PROV_END:
                ESP_LOGI(TAG_PROV, "Proces Provisioningu zakonczony. Wylaczam BLE (zwalniam RAM).");
                wifi_prov_mgr_deinit(); // Zwalnia pamięć po Bluetooth
                break;
            default:
                break;
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "=> SIEC GOTOWA! Uzyskano adres IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Utracono polaczenie. Probuje ponownie...");
        esp_wifi_connect();
    }
}

// ==========================================================
// 2. INICJALIZACJA WIFI I PROVISIONINGU
// ==========================================================
static void wifi_init_and_prov(void)
{
    // A. Standardowy szkielet WiFi w ESP-IDF
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // B. Rejestracja zdarzeń
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    // C. Konfiguracja Managera Provisioningu
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // D. Sprawdzenie, czy urządzenie ma już wpisane hasło
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        ESP_LOGI(TAG_PROV, "Brak danych WiFi w pamieci. Uruchamiam proces Provisioningu...");
        
        // ZMIANA: Zabezpieczenie SECURITY_1 (wymaga podania PINu w aplikacji!)
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        
        // Definiujemy nasz kod PIN (Proof of Possession)
        // Wpiszesz go w aplikacji w polu "PoP"
        const char *pop = "12345678"; 
        
        // Start provisioningu BLE z naszym PINem
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)pop, "PROV_SmartHome", NULL));
    } else {
        ESP_LOGI(TAG_PROV, "Znaleziono konfiguracje WiFi w pamieci. Startuje tryb Station.");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        
        // Skoro mamy już sieć, zwalniamy moduł BLE
        wifi_prov_mgr_deinit();
    }
}

// ==========================================================
// 3. LOGIKA SPECYFICZNA DLA MODUŁÓW
// ==========================================================
#ifdef MODULE_SECURITY
void setupModule() { ESP_LOGI(TAG, "Inicjalizacja modulu: SECURITY"); }
#elif defined(MODULE_ACCESS)
void setupModule() { ESP_LOGI(TAG, "Inicjalizacja modulu: ACCESS"); }
#elif defined(MODULE_ENV)
void setupModule() { ESP_LOGI(TAG, "Inicjalizacja modulu: ENVIRONMENT"); }
#else
void setupModule() { ESP_LOGI(TAG, "Inicjalizacja modulu: DEFAULT (Czysty start)"); }
#endif

// ==========================================================
// GŁÓWNA FUNKCJA
// ==========================================================
extern "C" void app_main() {
    ESP_LOGI(TAG, "=== Start SmartHome Node (ESP-IDF) ===");

    // 1. Inicjalizacja NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Setup konkretnego modułu
    setupModule();

    // 3. Uruchomienie połączenia (odczyta dane lub odpali Bluetooth)
    wifi_init_and_prov();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}