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

// --- Biblioteki wymagane do obsługi OTA ---
#include "esp_ota_ops.h"
#include "esp_http_client.h"

static const char *TAG = "MAIN";
static const char *TAG_PROV = "PROV";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_MDNS = "MDNS";
static const char *TAG_OTA  = "OTA";

#define RPI_HOSTNAME "rpi-smarthome"
#define BROKER_USER  "esp32"
#define BROKER_PASS  "esp32"

static esp_mqtt_client_handle_t mqtt_client;
static bool mdns_initialized = false;

// ==========================================================
// 1. SILNIK AKTUALIZACJI BEZPRZEWODOWEJ (HTTP OTA)
// ==========================================================
static void ota_update_task(void *pvParameter)
{
    char *ota_url = (char *)pvParameter;
    ESP_LOGW(TAG_OTA, "Rozpoczynam pobieranie aktualizacji z adresu: %s", ota_url);

    esp_http_client_config_t config = {};
    config.url = ota_url;
    config.timeout_ms = 8000;
    config.skip_cert_common_name_check = true; // Dla lokalnych serwerów HTTP

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "Blad: Nie mozna otworzyc polaczenia HTTP z serwerem aktualizacji.");
        free(ota_url);
        vTaskDelete(NULL);
    }
    esp_http_client_fetch_headers(client);

    // Znajdujemy wolny bank pamięci (nieaktywny)
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG_OTA, "Zapisuje firmware do partycji: %s na adresie 0x%lx", update_partition->label, update_partition->address);

    esp_ota_handle_t update_handle = 0;
    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle));

    char ota_buffer[1024];
    int read_len;
    int total_read = 0;

    // Pętla pobierania strumieniowego binarki i zapisywania we flashu
    while ((read_len = esp_http_client_read(client, ota_buffer, sizeof(ota_buffer))) > 0) {
        esp_ota_write(update_handle, (const void *)ota_buffer, read_len);
        total_read += read_len;
        if (total_read % 51200 == 0) { // Log co 50 KB progresu
            ESP_LOGI(TAG_OTA, "Pobrano: %d bajtow...", total_read);
        }
    }

    ESP_LOGI(TAG_OTA, "Pobieranie zakonczone. Lacznie: %d bajtow. Weryfikacja...", total_read);

    if (esp_ota_end(update_handle) == ESP_OK) {
        // Przełączamy flagę bootloadera na nową partycję!
        if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
            ESP_LOGW(TAG_OTA, "SUKCES! Aktualizacja zainstalowana poprawnie. Restartuje ESP32...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        } else {
            ESP_LOGE(TAG_OTA, "Blad przelaczania partycji startowej.");
        }
    } else {
        ESP_LOGE(TAG_OTA, "Blad: Sprawdzenie sumy kontrolnej pobranego pliku nie powiodlo sie.");
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(ota_url);
    vTaskDelete(NULL);
}

// ==========================================================
// 2. DYNAMICZNE ODNAJDYWANIE IP MALINKI (mDNS Resolution)
// ==========================================================
static esp_err_t resolve_rpi_ip(char *out_ip_uri, size_t max_len)
{
    if (!mdns_initialized) {
        esp_err_t err = mdns_init();
        if (err == ESP_OK) {
            mdns_initialized = true;
            mdns_hostname_set("smarthome-node");
        } else {
            return err;
        }
    }
    esp_ip4_addr_t addr = { .addr = 0 };
    esp_err_t err = mdns_query_a(RPI_HOSTNAME, 4000, &addr);
    if (err == ESP_OK) {
        snprintf(out_ip_uri, max_len, "mqtt://" IPSTR ":1883", IP2STR(&addr));
        return ESP_OK;
    }
    return ESP_FAIL;
}

// ==========================================================
// 3. OBSŁUGA ZDARZEŃ MQTT
// ==========================================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "POMYSLNIE polaczono z dynamicznym Brokerem MQTT!");
            // Subskrybujemy globalny kanał aktualizacji systemowych
            esp_mqtt_client_subscribe(client, "home/system/update", 1);
            esp_mqtt_client_publish(client, "home/system/status", "ONLINE", 0, 1, 0);
            break;

        case MQTT_EVENT_DATA:
            // Sprawdzamy czy przyszła prośba o aktualizację bezprzewodową
            if (strncmp(event->topic, "home/system/update", event->topic_len) == 0) {
                // Kopiujemy payload (URL), bo event zniknie po wyjściu z tej funkcji
                char *url_payload = (char *)malloc(event->data_len + 1);
                snprintf(url_payload, event->data_len + 1, "%.*s", event->data_len, event->data);
                
                ESP_LOGW(TAG_MQTT, "Otrzymano zadanie OTA z adresem serwera. Tworze bezpieczny watek...");
                xTaskCreate(ota_update_task, "ota_update_task", 8192, (void *)url_payload, 5, NULL);
            }
            break;
            
        default:
            break;
    }
}

static void mqtt_app_start(const char *broker_uri)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.credentials.username = BROKER_USER;
    mqtt_cfg.credentials.authentication.password = BROKER_PASS;
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

static void mqtt_discovery_task(void *pvParameters)
{
    char dynamic_broker_uri[64];
    while (1) {
        if (resolve_rpi_ip(dynamic_broker_uri, sizeof(dynamic_broker_uri)) == ESP_OK) {
            mqtt_app_start(dynamic_broker_uri);
            vTaskDelete(NULL); 
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ==========================================================
// 4. OBSŁUGA WIFI I SEKCJE STARTOWE
// ==========================================================
static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT && event_id == WIFI_PROV_END) {
        wifi_prov_mgr_deinit();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xTaskCreate(mqtt_discovery_task, "mqtt_discovery_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    }
}

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

    wifi_prov_mgr_config_t config = {};
    config.scheme = wifi_prov_scheme_ble;
    config.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE;
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)"12345678", "PROV_SmartHome", NULL));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_prov_mgr_deinit();
    }
}

void setupModule() { 
    ESP_LOGW(TAG, "=============================================");
    ESP_LOGW(TAG, "Wersja v2.0-ONLINE ");
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