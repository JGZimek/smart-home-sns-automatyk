#include "mqtt_core.h"
#include "network_core.h"
#include "module_config.h"
#include "device_core.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"

// Dołączamy nagłówki sprzętowe
#if defined(MODULE_SECURITY)
    #include "security_system.h"
#elif defined(MODULE_ENV)
    #include "environment_system.h"
#elif defined(MODULE_ACCESS)
    #include "access_system.h"
#endif

static const char *TAG_MQTT = "MQTT";
static esp_mqtt_client_handle_t mqtt_client = NULL; // Zainicjowane jako NULL

// --- Ponowne wykrywanie brokera (plug-and-play) ---
// Gdy broker długo jest nieosiągalny (zgaszony / zmienił IP), po serii
// nieudanych prób niszczymy klienta i ponawiamy wyszukiwanie przez mDNS.
#define MQTT_MAX_FAILS_BEFORE_REDISCOVER 10
static TaskHandle_t s_reconnect_task = NULL;
static int s_mqtt_fail_cnt = 0;  // dostęp wyłącznie z wątku zdarzeń MQTT

// Teardown wykonywany POZA wątkiem zdarzeń MQTT (nie wolno niszczyć klienta
// z jego własnego callbacku).
static void mqtt_reconnect_task(void *arg) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (mqtt_client != NULL) {
            esp_mqtt_client_stop(mqtt_client);
            esp_mqtt_client_destroy(mqtt_client);
            mqtt_client = NULL;
        }
        ESP_LOGW(TAG_MQTT, "Broker nieosiagalny — ponawiam wykrywanie przez mDNS.");
        xTaskCreate(mqtt_discovery_task, "mqtt_discovery_task", 4096, NULL, 3, NULL);
    }
}

// NOWA FUNKCJA: Globalny publikator MQTT
void mqtt_publish(const char* topic, const char* payload, int qos, int retain) {
    if (mqtt_client != NULL) {
        esp_mqtt_client_publish(mqtt_client, topic, payload, strlen(payload), qos, retain);
    }
}

// Dokładne porównanie tematu zdarzenia z oczekiwanym (esp-mqtt nie kończy
// event->topic znakiem NUL i podaje jego długość osobno).
static bool mqtt_topic_equals(esp_mqtt_event_handle_t event, const char *expected) {
    size_t len = strlen(expected);
    return (size_t)event->topic_len == len && strncmp(event->topic, expected, len) == 0;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "POMYSLNIE polaczono z dynamicznym Brokerem MQTT!");
            s_mqtt_fail_cnt = 0;
            network_notify_broker_connected();   // wyłącza watchdog provisioningu
            esp_mqtt_client_subscribe(client, OTA_TOPIC, 1);
            esp_mqtt_client_subscribe(client, CMD_TOPIC, 1);   // wspólny kanał komend (reboot/reset_wifi/...)
            // Retained ONLINE; broker automatycznie nadpisze go LWT "OFFLINE" przy zerwaniu
            esp_mqtt_client_publish(client, AVAILABILITY_TOPIC, "ONLINE", 0, 1, 1);
            // Retained „wizytówka" węzła – pozwala backendowi auto-wykryć urządzenie
            device_publish_info();

            // Subskrypcja nasłuchu sprzętowego zależnie od modułu
            #if defined(MODULE_SECURITY)
                esp_mqtt_client_subscribe(client, "home/security/arm/set", 1);
            #elif defined(MODULE_ENV)
                esp_mqtt_client_subscribe(client, "home/garden/fan/+/set", 1);
            #elif defined(MODULE_ACCESS)
                esp_mqtt_client_subscribe(client, "home/access/door/set", 1);
            #endif
            break;

        case MQTT_EVENT_DISCONNECTED:
        case MQTT_EVENT_ERROR:
            if (++s_mqtt_fail_cnt >= MQTT_MAX_FAILS_BEFORE_REDISCOVER && s_reconnect_task != NULL) {
                s_mqtt_fail_cnt = 0;
                xTaskNotifyGive(s_reconnect_task); // teardown + rediscovery poza tym wątkiem
            }
            break;

        case MQTT_EVENT_DATA: { 
            
            // 1. Logika OTA
            if (mqtt_topic_equals(event, OTA_TOPIC)) {
                char *url_payload = (char *)malloc(event->data_len + 1);
                if (url_payload == NULL) {
                    ESP_LOGE(TAG_MQTT, "Brak pamieci na adres OTA.");
                    break;
                }
                snprintf(url_payload, event->data_len + 1, "%.*s", event->data_len, event->data);

                ESP_LOGW(TAG_MQTT, "Otrzymano zadanie OTA z adresem serwera. Tworze bezpieczny watek...");
                if (xTaskCreate(ota_update_task, "ota_update_task", 8192, (void *)url_payload, 5, NULL) != pdPASS) {
                    ESP_LOGE(TAG_MQTT, "Nie udalo sie utworzyc watku OTA.");
                    free(url_payload);
                }
                break; // wiadomosci OTA nie przekazujemy do logiki sprzetowej
            }

            // 1b. Wspólny kanał komend (diagnostyka / rekonfiguracja)
            if (mqtt_topic_equals(event, CMD_TOPIC)) {
                char cmd_buf[160];
                int c_len = (event->data_len < (int)sizeof(cmd_buf) - 1) ? event->data_len : (int)sizeof(cmd_buf) - 1;
                snprintf(cmd_buf, sizeof(cmd_buf), "%.*s", c_len, event->data);
                device_handle_command(cmd_buf, c_len);
                break;
            }

            // 2. Przekazanie danych do logiki sprzętowej
            #if defined(MODULE_SECURITY) || defined(MODULE_ENV) || defined(MODULE_ACCESS)
                char topic_buf[64];
                char data_buf[128];
                
                // Zabezpieczenie przed ew. przepełnieniem bufora
                int t_len = (event->topic_len < sizeof(topic_buf) - 1) ? event->topic_len : sizeof(topic_buf) - 1;
                int d_len = (event->data_len < sizeof(data_buf) - 1) ? event->data_len : sizeof(data_buf) - 1;
                
                snprintf(topic_buf, sizeof(topic_buf), "%.*s", t_len, event->topic);
                snprintf(data_buf, sizeof(data_buf), "%.*s", d_len, event->data);
                
                // Wywołanie właściwego callbacka
                #if defined(MODULE_SECURITY)
                    security_mqtt_callback(topic_buf, data_buf, event->data_len);
                #elif defined(MODULE_ENV)
                    environment_mqtt_callback(topic_buf, data_buf, event->data_len);
                #elif defined(MODULE_ACCESS)
                    access_mqtt_callback(topic_buf, data_buf, event->data_len);
                #endif
            #endif
            
            break;
        } 
            
        default:
            break;
    }
}

void mqtt_app_start(const char *broker_uri)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.credentials.username = BROKER_USER;
    mqtt_cfg.credentials.authentication.password = BROKER_PASS;

    // Last Will & Testament: broker rozgłosi OFFLINE, gdy węzeł zniknie z sieci.
    // msg_len = 0 -> esp-mqtt użyje strlen() dla łańcucha tekstowego.
    mqtt_cfg.session.last_will.topic = AVAILABILITY_TOPIC;
    mqtt_cfg.session.last_will.msg = "OFFLINE";
    mqtt_cfg.session.last_will.msg_len = 0;
    mqtt_cfg.session.last_will.qos = 1;
    mqtt_cfg.session.last_will.retain = 1;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // Jednorazowo: zadanie obsługujące ponowne wykrywanie brokera (plug-and-play).
    if (s_reconnect_task == NULL) {
        xTaskCreate(mqtt_reconnect_task, "mqtt_reconnect", 4096, NULL, 4, &s_reconnect_task);
    }
}

void mqtt_discovery_task(void *pvParameters)
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