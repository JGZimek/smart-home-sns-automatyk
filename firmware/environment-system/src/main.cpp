#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Adafruit_BME280.h>
#include <time.h>
#include "esp_log.h"

// --- TAGI LOGÓW ---
static const char* TAG_MAIN = "ENV_SYS";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_SENS = "SENS";
static const char* TAG_TIME = "TIME";

// --- KONFIGURACJA PINÓW ---
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22

// --- USTAWIENIA ---
#define SENSOR_INTERVAL 5000
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // UTC+1
const int   daylightOffset_sec = 3600; // UTC+2 (Lato)

// --- OBIEKTY GLOBALNE ---
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
Adafruit_BME280 bme;

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
char mqtt_port[6] = "1883";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32";

// --- STRUKTURY DANYCH ---
enum SensorType { TEMP, HUM, PRES };

struct SensorMeasurement {
    SensorType type;
    float value;
    time_t timestamp;
};

QueueHandle_t msgQueue;

// --- FUNKCJE POMOCNICZE ---

// Callback Wi-Fi Managera przy zapisie konfiguracji
void saveParamsCallback() {
    ESP_LOGI(TAG_WIFI, "Saving configuration from captive portal...");
    String ip_str = wm.server->arg("mqtt_ip");
    if (ip_str.length() > 0) {
        ESP_LOGI(TAG_WIFI, "New MQTT IP: %s", ip_str.c_str());
        ip_str.toCharArray(mqtt_server, 40);
        preferences.begin("smarthome", false);
        preferences.putString("mqtt_ip", mqtt_server);
        preferences.end();
    }
}

// Handler zdarzeń Wi-Fi (dla lepszego debugowania błędów sieci)
void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            ESP_LOGI(TAG_WIFI, "Connected! IP Address: %s", WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            ESP_LOGW(TAG_WIFI, "Disconnected from WiFi! Attempting reconnection...");
            break;
        default:
            break;
    }
}

void printMqttError(int state) {
    switch (state) {
        case -4: ESP_LOGE(TAG_MQTT, "Connection Timeout"); break;
        case -3: ESP_LOGE(TAG_MQTT, "Connection Lost"); break;
        case -2: ESP_LOGE(TAG_MQTT, "Connect Failed (Check IP/Network)"); break;
        case -1: ESP_LOGW(TAG_MQTT, "Disconnected"); break;
        case 1:  ESP_LOGE(TAG_MQTT, "Bad Protocol"); break;
        case 2:  ESP_LOGE(TAG_MQTT, "Bad Client ID"); break;
        case 3:  ESP_LOGE(TAG_MQTT, "Unavailable"); break;
        case 4:  ESP_LOGE(TAG_MQTT, "Bad Credentials"); break;
        case 5:  ESP_LOGE(TAG_MQTT, "Unauthorized"); break;
        default: ESP_LOGE(TAG_MQTT, "Unknown Error: %d", state); break;
    }
}

void reconnectMqtt() {
    if (!client.connected()) {
        ESP_LOGI(TAG_MQTT, "Connecting to broker at %s...", mqtt_server);
        
        String clientId = "EnvSystem-" + String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            ESP_LOGI(TAG_MQTT, "Connected successfully!");
            client.publish("home/garden/system/status", "ONLINE");
        } else {
            printMqttError(client.state());
            // Opóźnienie jest robione w pętli taska, tu tylko logujemy błąd
        }
    }
}

void initTime() {
    ESP_LOGI(TAG_TIME, "Syncing NTP time...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    struct tm timeinfo;
    int retry = 0;
    while(!getLocalTime(&timeinfo) && retry < 20) {
        delay(500);
        retry++;
    }
    
    if (retry >= 20) {
        ESP_LOGE(TAG_TIME, "NTP Sync Failed! Timestamps will be incorrect.");
    } else {
        char timeStr[64];
        strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
        ESP_LOGI(TAG_TIME, "Time synchronized: %s", timeStr);
    }
}

// ---------------------------------------------------------
// TASK 1: SENSOR & CONTROL (Core 1)
// ---------------------------------------------------------
void sensorControlTask(void * parameter) {
    ESP_LOGI(TAG_SENS, "Task started on Core 1");

    if (!bme.begin(0x76)) {
        ESP_LOGE(TAG_SENS, "BME280 not found! Check wiring (SDA/SCL).");
    } else {
        ESP_LOGI(TAG_SENS, "BME280 initialized OK.");
    }
    
    for(;;) {
        time_t now;
        time(&now); 

        float temp = bme.readTemperature();
        float hum = bme.readHumidity();
        float pres = bme.readPressure() / 100.0F;

        // Logowanie debugowe (opcjonalne - widoczne tylko przy CORE_DEBUG_LEVEL >= 4)
        ESP_LOGD(TAG_SENS, "Read: T=%.1f H=%.1f P=%.1f", temp, hum, pres);

        SensorMeasurement m;
        m.timestamp = now;

        // Temperatura
        m.type = TEMP; m.value = temp;
        if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) {
             ESP_LOGW(TAG_SENS, "Queue full! Dropping TEMP packet.");
        }

        // Wilgotność
        m.type = HUM; m.value = hum;
        xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10));

        // Ciśnienie
        m.type = PRES; m.value = pres;
        xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10));

        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL));
    }
}

// ---------------------------------------------------------
// TASK 2: NETWORK (Core 0)
// ---------------------------------------------------------
void networkTask(void * parameter) {
    ESP_LOGI(TAG_MQTT, "Task started on Core 0");
    
    client.setServer(mqtt_server, 1883);
    SensorMeasurement inMsg;
    char topicBuffer[64];
    char payloadBuffer[128];

    for(;;) {
        // 1. WiFi Check
        if (WiFi.status() != WL_CONNECTED) {
             // Logowanie jest w WiFiEvent, tutaj tylko czekamy
             vTaskDelay(pdMS_TO_TICKS(2000));
             continue;
        }

        // 2. MQTT Check
        if (!client.connected()) {
            reconnectMqtt();
            if (!client.connected()) {
                 vTaskDelay(pdMS_TO_TICKS(5000)); // Czekaj przed kolejną próbą
                 continue;
            }
        }
        client.loop();

        // 3. Queue Processing
        if (xQueueReceive(msgQueue, &inMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            switch(inMsg.type) {
                case TEMP: snprintf(topicBuffer, 64, "home/garden/environment/temperature"); break;
                case HUM:  snprintf(topicBuffer, 64, "home/garden/environment/humidity"); break;
                case PRES: snprintf(topicBuffer, 64, "home/garden/environment/pressure"); break;
            }

            snprintf(payloadBuffer, 128, "{\"value\": %.2f, \"ts\": %ld}", inMsg.value, inMsg.timestamp);

            if (client.publish(topicBuffer, payloadBuffer)) {
                ESP_LOGI(TAG_MQTT, "Sent %s: %.2f", topicBuffer, inMsg.value);
            } else {
                ESP_LOGE(TAG_MQTT, "Publish failed! (Client busy or disconn?)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200);
    
    // Ustawiamy poziom logowania na INFO (można zmienić na VERBOSE/DEBUG/ERROR)
    esp_log_level_set("*", ESP_LOG_INFO); 
    
    delay(1000);
    ESP_LOGI(TAG_MAIN, "System Startup - Environment System");

    // 1. Load Config
    preferences.begin("smarthome", true);
    String saved_ip = preferences.getString("mqtt_ip", "");
    if (saved_ip.length() > 0) {
        saved_ip.toCharArray(mqtt_server, 40);
        ESP_LOGI(TAG_MAIN, "Loaded MQTT IP: %s", mqtt_server);
    } else {
        ESP_LOGW(TAG_MAIN, "No MQTT IP saved. Using default.");
    }
    preferences.end();

    // 2. WiFi Manager
    // Rejestracja eventu Wi-Fi dla lepszego logowania
    WiFi.onEvent(WiFiEvent);

    WiFiManagerParameter custom_mqtt_ip("mqtt_ip", "MQTT Broker IP", mqtt_server, 40);
    wm.addParameter(&custom_mqtt_ip);
    wm.setSaveParamsCallback(saveParamsCallback);
    
    // wm.resetSettings(); // Włącz to RAZ, jeśli chcesz wyczyścić zapisane WiFi, potem zakomentuj
    
    if(!wm.autoConnect("SmartHome-Env")) {
        ESP_LOGE(TAG_WIFI, "Failed to connect. Restarting...");
        ESP.restart();
    }
    
    // 3. Init Time
    initTime();

    // 4. Create Queue
    msgQueue = xQueueCreate(20, sizeof(SensorMeasurement));
    if (msgQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create queue! Halt.");
        while(1); // Stop if no memory
    }

    // 5. Start Tasks
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorControlTask, "SensTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}