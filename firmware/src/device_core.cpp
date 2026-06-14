#include "device_core.h"
#include "module_config.h"
#include "mqtt_core.h"
#include "network_core.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "cJSON.h"

static const char* TAG_DEV = "DEVICE";

#define DEVICE_DIAG_INTERVAL_S 30   // co ile sekund publikowany jest heartbeat

// --- Identyfikator z adresu MAC (stały dla danej płytki) ---
const char* device_id(void) {
    static char id[20] = {0};
    if (id[0] == 0) {
        uint8_t mac[6] = {0};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(id, sizeof(id), "esp32-%02x%02x%02x", mac[3], mac[4], mac[5]);
    }
    return id;
}

static const char* reset_reason_str(void) {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        default:                return "other";
    }
}

static void get_ip_str(char* out, size_t len) {
    out[0] = 0;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (netif != NULL && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        snprintf(out, len, IPSTR, IP2STR(&ip.ip));
    }
}

// Retained „wizytówka" węzła – backend odczytuje ją natychmiast po subskrypcji.
void device_publish_info(void) {
    char ip[16];
    get_ip_str(ip, sizeof(ip));
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"kind\":\"%s\",\"name\":\"%s\",\"fw\":\"%s\",\"idf\":\"%s\",\"ip\":\"%s\","
        "\"cmd_topic\":\"%s\",\"diag_topic\":\"%s\"}",
        device_id(), DEVICE_KIND, MODULE_NAME, FIRMWARE_VERSION, esp_get_idf_version(), ip,
        CMD_TOPIC, DIAG_TOPIC);
    mqtt_publish(INFO_TOPIC, payload, 1, 1);
}

void device_publish_diag(void) {
    wifi_ap_record_t ap;
    int rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"uptime_s\":%lld,\"heap\":%u,\"min_heap\":%u,\"rssi\":%d,\"reset\":\"%s\"}",
        device_id(),
        (long long)(esp_timer_get_time() / 1000000),
        (unsigned)esp_get_free_heap_size(),
        (unsigned)esp_get_minimum_free_heap_size(),
        rssi, reset_reason_str());
    mqtt_publish(DIAG_TOPIC, payload, 0, 0);
}

static void diag_task(void* arg) {
    // mqtt_publish jest bezpieczne przed połączeniem (no-op gdy klient == NULL),
    // więc zadanie może startować od razu – pierwsze pakiety wyjdą po połączeniu.
    while (1) {
        device_publish_diag();
        vTaskDelay(pdMS_TO_TICKS(DEVICE_DIAG_INTERVAL_S * 1000));
    }
}

void device_start_diag_task(void) {
    xTaskCreate(diag_task, "diag_task", 4096, NULL, 2, NULL);
}

// Akceptuje proste słowa kluczowe ("reboot") lub JSON ({"cmd":"reboot"}) –
// dopasowanie po wystąpieniu podciągu, więc backend ma dowolność formatu.
bool device_handle_command(const char* data, int data_len) {
    if (strstr(data, "reset_wifi") != NULL) {
        ESP_LOGW(TAG_DEV, "Komenda: reset_wifi -> czyszcze poswiadczenia i restartuje (BLE provisioning)");
        mqtt_publish(DIAG_TOPIC, "{\"ack\":\"reset_wifi\"}", 1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_wifi_restore();   // kasuje zapisaną sieć Wi-Fi z NVS
        esp_restart();
        return true;          // nieosiągalne
    }
    // set_wifi: zdalne przepiecie na nowa siec. Format: {"cmd":"set_wifi","ssid":"...","pass":"..."}
    // Sprawdzane PO reset_wifi (bo "reset_wifi" zawiera podciag "set_wifi").
    if (strstr(data, "set_wifi") != NULL) {
        cJSON *root = cJSON_Parse(data);
        if (root != NULL) {
            const cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
            const cJSON *pass = cJSON_GetObjectItem(root, "pass");
            if (cJSON_IsString(ssid) && ssid->valuestring != NULL) {
                const char *p = (cJSON_IsString(pass) && pass->valuestring) ? pass->valuestring : "";
                ESP_LOGW(TAG_DEV, "Komenda: set_wifi -> nowa siec SSID '%s', restart...", ssid->valuestring);
                mqtt_publish(DIAG_TOPIC, "{\"ack\":\"set_wifi\"}", 1, 0);
                wifi_set_credentials(ssid->valuestring, p);  // zapis do NVS
                cJSON_Delete(root);
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();                                // czyste przejscie na nowa siec
                return true;                                  // nieosiagalne
            }
            cJSON_Delete(root);
        }
        ESP_LOGW(TAG_DEV, "set_wifi: oczekiwany JSON {\"cmd\":\"set_wifi\",\"ssid\":\"...\",\"pass\":\"...\"}");
        return false;
    }
    if (strstr(data, "reboot") != NULL) {
        ESP_LOGW(TAG_DEV, "Komenda: reboot");
        mqtt_publish(DIAG_TOPIC, "{\"ack\":\"reboot\"}", 1, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        return true;
    }
    if (strstr(data, "identify") != NULL) {
        ESP_LOGW(TAG_DEV, "Komenda: identify (%s / %s)", MODULE_NAME, device_id());
        device_publish_info();
        return true;
    }
    if (strstr(data, "diag") != NULL || strstr(data, "ping") != NULL) {
        device_publish_diag();
        return true;
    }
    ESP_LOGW(TAG_DEV, "Nieznana komenda: %.*s", data_len, data);
    return false;
}
