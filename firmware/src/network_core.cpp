#include "network_core.h"
#include "mqtt_core.h"
#include "module_config.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "mdns.h" 
#include "esp_ota_ops.h"
#include "esp_http_client.h"

static const char *TAG_PROV = "PROV";
// static const char *TAG_MDNS = "MDNS";
static const char *TAG_OTA  = "OTA";

static bool mdns_initialized = false;
static int s_retry_cnt = 0;
#define MAX_RETRY_BEFORE_FALLBACK 5
static bool s_provisioning_active = false;
static bool s_prov_mgr_inited = false;        // czy menedżer provisioningu jest zainicjowany
static bool s_broker_connected = false;       // czy KIEDYKOLWIEK od startu udało się połączyć z brokerem
static bool s_broker_watchdog_started = false;

// Po ilu sekundach od uzyskania IP — gdy broker NIGDY nie był osiągalny — wrócić do BLE
// provisioningu. Chroni przed utknięciem na sieci bez brokera (np. po zmianie lokalizacji
// albo gdy zapisana jest stara, nieaktualna sieć). NIE dotyczy węzła, który raz połączył
// się z brokerem, więc chwilowa awaria brokera na działającej makiecie nie wymusza parowania.
#define BROKER_PROV_TIMEOUT_S 180

void ota_update_task(void *pvParameter)
{
    char *ota_url = (char *)pvParameter;
    ESP_LOGW(TAG_OTA, "Rozpoczynam pobieranie aktualizacji z adresu: %s", ota_url);

    esp_http_client_config_t config = {};
    config.url = ota_url;
    config.timeout_ms = 8000;
    config.skip_cert_common_name_check = true;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG_OTA, "Blad: Nie mozna utworzyc klienta HTTP.");
        free(ota_url);
        vTaskDelete(NULL);
        return;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG_OTA, "Blad: Nie mozna otworzyc polaczenia HTTP z serwerem aktualizacji.");
        esp_http_client_cleanup(client);
        free(ota_url);
        vTaskDelete(NULL);
        return;
    }
    esp_http_client_fetch_headers(client);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG_OTA, "Blad: Brak wolnej partycji OTA.");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(ota_url);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_OTA, "Zapisuje firmware do partycji: %s na adresie 0x%lx", update_partition->label, update_partition->address);

    esp_ota_handle_t update_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        // Nie używamy ESP_ERROR_CHECK, aby błędna aktualizacja nie restartowała całego węzła.
        ESP_LOGE(TAG_OTA, "esp_ota_begin nie powiodlo sie: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        free(ota_url);
        vTaskDelete(NULL);
        return;
    }

    char ota_buffer[1024];
    int read_len;
    int total_read = 0;
    int next_log = 51200;
    bool write_ok = true;

    while ((read_len = esp_http_client_read(client, ota_buffer, sizeof(ota_buffer))) > 0) {
        if (esp_ota_write(update_handle, (const void *)ota_buffer, read_len) != ESP_OK) {
            ESP_LOGE(TAG_OTA, "Blad zapisu do partycji OTA.");
            write_ok = false;
            break;
        }
        total_read += read_len;
        if (total_read >= next_log) {
            ESP_LOGI(TAG_OTA, "Pobrano: %d bajtow...", total_read);
            next_log += 51200;
        }
    }
    if (read_len < 0) {
        ESP_LOGE(TAG_OTA, "Blad odczytu strumienia HTTP.");
        write_ok = false;
    }

    if (!write_ok) {
        esp_ota_abort(update_handle);
        ESP_LOGE(TAG_OTA, "Aktualizacja przerwana. Wezel pozostaje na dotychczasowej wersji.");
    } else {
        ESP_LOGI(TAG_OTA, "Pobieranie zakonczone. Lacznie: %d bajtow. Weryfikacja...", total_read);
        err = esp_ota_end(update_handle);
        if (err == ESP_OK && esp_ota_set_boot_partition(update_partition) == ESP_OK) {
            ESP_LOGW(TAG_OTA, "SUKCES! Aktualizacja zainstalowana poprawnie. Restartuje ESP32...");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            free(ota_url);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        } else {
            ESP_LOGE(TAG_OTA, "Weryfikacja obrazu lub przelaczenie partycji nie powiodlo sie: %s", esp_err_to_name(err));
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(ota_url);
    vTaskDelete(NULL);
}

esp_err_t resolve_rpi_ip(char *out_ip_uri, size_t max_len)
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
        snprintf(out_ip_uri, max_len, "mqtt://" IPSTR ":%d", IP2STR(&addr), MQTT_BROKER_PORT);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Uruchamia BLE provisioning, w razie potrzeby reinicjalizując menedżer (po uzyskaniu IP
// jest on zwalniany dla oszczędności RAM). Zapisane poświadczenia NIE są kasowane —
// dopiero podanie nowej sieci w aplikacji je nadpisze.
static void start_ble_provisioning(void)
{
    if (s_provisioning_active) return;
    s_provisioning_active = true;

    if (!s_prov_mgr_inited) {
        wifi_prov_mgr_config_t config = {};
        config.scheme = wifi_prov_scheme_ble;
        config.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE;
        ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
        s_prov_mgr_inited = true;
    }
    ESP_LOGW(TAG_PROV, "Uruchamiam BLE provisioning (%s). Mozna wskazac nowa siec Wi-Fi.", PROV_BLE_NAME);
    wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, (const void *)PROV_POP, PROV_BLE_NAME, NULL));
}

// Wołane przez warstwę MQTT po udanym połączeniu z brokerem — wyłącza watchdog provisioningu.
void network_notify_broker_connected(void)
{
    s_broker_connected = true;
}

// Watchdog: jeśli mimo połączenia z Wi-Fi broker pozostaje nieosiągalny i nigdy nie udało
// się z nim połączyć od startu, wracamy do BLE provisioningu, aby umożliwić zmianę sieci
// bez podłączania kabla.
static void broker_watchdog_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(BROKER_PROV_TIMEOUT_S * 1000));
    if (!s_broker_connected && !s_provisioning_active) {
        ESP_LOGE(TAG_PROV, "Broker nieosiagalny przez %ds mimo polaczenia z Wi-Fi. Wracam do BLE provisioning.", BROKER_PROV_TIMEOUT_S);
        start_ble_provisioning();
    }
    vTaskDelete(NULL);
}

static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_START) {
            s_provisioning_active = true;
        } else if (event_id == WIFI_PROV_END) {
            wifi_prov_mgr_deinit();
            s_prov_mgr_inited = false;
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (!s_provisioning_active) {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_cnt = 0;

        if (!s_provisioning_active) {
            ESP_LOGI(TAG_PROV, "Polaczono automatycznie z pamieci NVS. Zwalniam menedzer BLE.");
            wifi_prov_mgr_deinit();
            s_prov_mgr_inited = false;
        }

        // Watchdog brokera startuje raz, po pierwszym uzyskaniu IP.
        if (!s_broker_watchdog_started) {
            s_broker_watchdog_started = true;
            xTaskCreate(broker_watchdog_task, "broker_wd", 3072, NULL, 3, NULL);
        }

        xTaskCreate(mqtt_discovery_task, "mqtt_discovery_task", 4096, NULL, 3, NULL);

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!s_provisioning_active) {
            if (s_retry_cnt < MAX_RETRY_BEFORE_FALLBACK) {
                s_retry_cnt++;
                ESP_LOGW(TAG_PROV, "Brak polaczenia ze znana siecia Wi-Fi. Proba: %d/%d...", s_retry_cnt, MAX_RETRY_BEFORE_FALLBACK);
                esp_wifi_connect();
            } else {
                ESP_LOGE(TAG_PROV, "Makieta stracila dostep do sieci! Uruchamiam awaryjny BLE Provisioning (%s).", PROV_BLE_NAME);
                start_ble_provisioning();
            }
        }
    }
}

void wifi_init_and_prov(void)
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
    s_prov_mgr_inited = true;

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        start_ble_provisioning();
    } else {
        ESP_LOGI(TAG_PROV, "Dane Wi-Fi obecne we flashu. Uruchamiam probe polaczenia...");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}