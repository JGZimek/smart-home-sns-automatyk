#include "mqtt_core.h"
#include "network_core.h"
#include "module_config.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG_MQTT = "MQTT";
static esp_mqtt_client_handle_t mqtt_client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "POMYSLNIE polaczono z dynamicznym Brokerem MQTT!");
            esp_mqtt_client_subscribe(client, OTA_TOPIC, 1);
            esp_mqtt_client_publish(client, "home/system/status", "ONLINE", 0, 1, 0);
            break;

        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, OTA_TOPIC, event->topic_len) == 0) {
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

void mqtt_app_start(const char *broker_uri)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri;
    mqtt_cfg.credentials.username = BROKER_USER;
    mqtt_cfg.credentials.authentication.password = BROKER_PASS;
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
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