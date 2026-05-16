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

// --- Oficjalne biblioteki ESP-IDF ---
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "mqtt_client.h"  
#include "mdns.h"

static const char *TAG = "MAIN";
static const char *TAG_PROV = "PROV";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_MDNS = "MDNS";

// Nazwa hosta Twojego Raspberry Pi (z logów SSH: rpi-smarthome)
#define RPI_HOSTNAME "rpi-smarthome"
#define BROKER_USER  "esp32"
#define BROKER_PASS  "esp32"

static esp_mqtt_client_handle_t mqtt_client;
static bool mdns_initialized = false;

// ==========================================================
// 1. DYNAMICZNE ODNAJDYWANIE IP MALINKI (mDNS Resolution)
// ==========================================================
static esp_err_t resolve_rpi_ip(char *out_ip_uri, size_t max_len)
{
    if (!mdns_initialized) {
        esp_err_t err = mdns_init();
        if (err == ESP_OK) {
            mdns_initialized = true;
            // Nadajemy nazwę sieciową temu modułowi ESP32
            mdns_hostname_set("smarthome-node");
            mdns_instance_name_set("SmartHome ESP32 Node");
        } else {
            ESP_LOGE(TAG_MDNS, "Blad inicjalizacji mDNS: %d", err);
            return err;
        }
    }

    esp_ip4_addr_t addr;
    addr.addr = 0;

    ESP_LOGI(TAG_MDNS, "Wyszukiwanie przez mDNS adresu dla: %s.local...", RPI_HOSTNAME);
    
    // Odpytanie o rekord typu A (IPv4) z timeoutem 4 sekund
    esp_err_t err = mdns_query_a(RPI_HOSTNAME, 4000, &addr);
    if (err == ESP_OK) {
        snprintf(out_ip_uri, max_len, "mqtt://" IPSTR ":1883", IP2STR(&addr));
        ESP_LOGI(TAG_MDNS, "SUKCES! Odnaleziono ruterem przypisane IP malinki: %s", out_ip_uri);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG_MDNS, "mDNS nie odpowiedzial. Nie udalo sie znalezc %s.local.", RPI_HOSTNAME);
        return ESP_FAIL;
    }
}

// ==========================================================
// 2. OBSŁUGA ZDARZEŃ MQTT
// ==========================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "POMYSLNIE polaczono z dynamicznym Brokerem MQTT!");
            esp_mqtt_client_publish(client, "home/system/status", "ONLINE", 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG_MQTT, "Rozlaczono z Brokerem MQTT. Automatyczny powrot za chwile...");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "Otrzymano wiadomosc na temat: %.*s", event->topic_len, event->topic);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG_MQTT, "Blad polaczenia na warstwie transportowej MQTT.");
            break;
        default:
            break;
    }
}

static void mqtt_app_start(const char *broker_uri)
{
    ESP_LOGI(TAG_MQTT, "Uruchamianie silnika MQTT z adresem URI: %s...", broker_uri);
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.credentials.username = BROKER_USER;
    mqtt_cfg.credentials.authentication.password = BROKER_PASS;
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// Zadanie działające w tle, odpytujące mDNS do momentu znalezienia sieci
static void mqtt_discovery_task(void *pvParameters)
{
    char dynamic_broker_uri[64];
    ESP_LOGI(TAG, "Uruchomiono asynchroniczne zadanie namierzania malinki w sieci lokalnej...");
    
    while (1) {
        if (resolve_rpi_ip(dynamic_broker_uri, sizeof(dynamic_broker_uri)) == ESP_OK) {
            mqtt_app_start(dynamic_broker_uri);
            vTaskDelete(NULL); // Sukces - wątek sprząta sam siebie i kończy działanie
        }
        ESP_LOGW(TAG, "Malinka nie odpowiedziala. Kolejna proba za 5 sekund...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ==========================================================
// 3. OBSŁUGA ZDARZEŃ (WIFI I PROVISIONING)
// ==========================================================
static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_START) {
            ESP_LOGI(TAG_PROV, "===============================================");
            ESP_LOGI(TAG_PROV, " PROVISIONING AKTYWNY (BLE)!");
            ESP_LOGI(TAG_PROV, " Nazwa urzadzenia BLE: PROV_SmartHome");
            ESP_LOGI(TAG_PROV, "===============================================");
        } else if (event_id == WIFI_PROV_CRED_FAIL) {
            wifi_prov_mgr_reset_sm_state_on_failure();
        } else if (event_id == WIFI_PROV_END) {
            wifi_prov_mgr_deinit();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "=> SIEC GOTOWA! Uzyskano adres IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // Odpalamy bezpieczny wątek tła do wykrywania mDNS i startu MQTT
        xTaskCreate(mqtt_discovery_task, "mqtt_discovery_task", 4096, NULL, 3, NULL);
        
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    }
}

// ==========================================================
// 4. INICJALIZACJA WIFI I PROVISIONINGU
// ==========================================================
static void wifi_init_and_prov(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_prov_event_handler, NULL));

    // C++ style zero-initialization (Zabezpieczenie przed ostrzeżeniami kompilatora)
    wifi_prov_mgr_config_t config = {};
    config.scheme = wifi_prov_scheme_ble;
    config.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE;
    
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        ESP_LOGI(TAG_PROV, "Brak danych WiFi w pamieci. Uruchamiam proces Provisioningu...");
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        const char *pop = "12345678"; 
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)pop, "PROV_SmartHome", NULL));
    } else {
        ESP_LOGI(TAG_PROV, "Znaleziono konfiguracje WiFi w pamieci. Startuje tryb Station.");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_prov_mgr_deinit();
    }
}

// ==========================================================
// 5. LOGIKA SPECYFICZNA DLA MODUŁÓW 
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