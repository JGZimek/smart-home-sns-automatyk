#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <time.h>
#include "esp_log.h"

// --- TAGI LOGÓW ---
static const char* TAG_MAIN = "ENV_SYS";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_SENS = "SENS";
static const char* TAG_PWR  = "POWER";
static const char* TAG_TIME = "TIME";

// --- KONFIGURACJA PINÓW ---
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22

// --- ADRESY I2C ---
#define INA1_ADDR       0x45 // Watomierz 1 (Solar)
#define INA2_ADDR       0x44 // Watomierz 2 (Bateria)

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

Adafruit_INA219 pwr1(INA1_ADDR);
Adafruit_INA219 pwr2(INA2_ADDR);

volatile bool pwr1_connected = false;
volatile bool pwr2_connected = false;

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
char mqtt_port[6] = "1883";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32";

// --- STRUKTURY DANYCH ---
enum SensorType { TEMP, HUM, PRES, VOLT, CURR, WATT };

struct SensorMeasurement {
    SensorType type;
    uint8_t sourceId; // 0=BME, 1=Solar, 2=Battery
    float value;
    time_t timestamp;
};

QueueHandle_t msgQueue;

// --- FUNKCJE POMOCNICZE ---

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

void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            ESP_LOGI(TAG_WIFI, "Connected! IP Address: %s", WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            ESP_LOGW(TAG_WIFI, "Disconnected from WiFi! Attempting reconnection...");
            break;
        default: break;
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
        ESP_LOGE(TAG_TIME, "NTP Sync Failed!");
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

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // Init BME280
    if (!bme.begin(0x76, &Wire)) {
        ESP_LOGE(TAG_SENS, "BME280 not found!");
    } else {
        ESP_LOGI(TAG_SENS, "BME280 initialized OK.");
    }

    // Init Watomierz 1 (Solar)
    if (!pwr1.begin(&Wire)) {
        ESP_LOGE(TAG_PWR, "INA219 #1 (Addr 0x%X) Not Found", INA1_ADDR);
    } else {
        pwr1_connected = true;
        pwr1.setCalibration_32V_2A(); 
        ESP_LOGI(TAG_PWR, "INA219 #1 (Solar) Connected");
    }

    // Init Watomierz 2 (Battery)
    if (!pwr2.begin(&Wire)) {
        ESP_LOGE(TAG_PWR, "INA219 #2 (Addr 0x%X) Not Found", INA2_ADDR);
    } else {
        pwr2_connected = true;
        pwr2.setCalibration_32V_2A(); 
        ESP_LOGI(TAG_PWR, "INA219 #2 (Battery) Connected");
    }
    
    for(;;) {
        time_t now;
        time(&now); 

        // 1. Odczyt BME280
        float temp = bme.readTemperature();
        float hum = bme.readHumidity();
        float pres = bme.readPressure() / 100.0F;

        SensorMeasurement m;
        m.timestamp = now;
        m.sourceId = 0; // ID 0 = Środowisko

        m.type = TEMP; m.value = temp;
        if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env TEMP");
        
        m.type = HUM; m.value = hum; 
        if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env HUM");

        m.type = PRES; m.value = pres; 
        if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env PRES");

        // 2. Odczyt Watomierza 1 (Solar)
        if (pwr1_connected) {
            m.sourceId = 1;
            
            m.type = VOLT; m.value = pwr1.getBusVoltage_V(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar VOLT");

            m.type = CURR; m.value = pwr1.getCurrent_mA(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar CURR");

            m.type = WATT; m.value = pwr1.getPower_mW(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar WATT");
        }

        // 3. Odczyt Watomierza 2 (Battery)
        if (pwr2_connected) {
            m.sourceId = 2;
            
            m.type = VOLT; m.value = pwr2.getBusVoltage_V(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt VOLT");

            m.type = CURR; m.value = pwr2.getCurrent_mA(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt CURR");

            m.type = WATT; m.value = pwr2.getPower_mW(); 
            if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt WATT");
        }

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
        if (WiFi.status() != WL_CONNECTED) { 
            vTaskDelay(pdMS_TO_TICKS(2000)); 
            continue; 
        }

        if (!client.connected()) {
            reconnectMqtt();
            if (!client.connected()) { 
                vTaskDelay(pdMS_TO_TICKS(5000)); 
                continue; 
            }
        }
        client.loop();

        if (xQueueReceive(msgQueue, &inMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            // --- TWORZENIE TEMATÓW ---
            if (inMsg.sourceId == 0) {
                // Dane środowiskowe (BME280)
                switch(inMsg.type) {
                    case TEMP: snprintf(topicBuffer, 64, "home/garden/environment/temperature"); break;
                    case HUM:  snprintf(topicBuffer, 64, "home/garden/environment/humidity"); break;
                    case PRES: snprintf(topicBuffer, 64, "home/garden/environment/pressure"); break;
                    default:   snprintf(topicBuffer, 64, "home/garden/environment/log"); break;
                }
            } else {
                // Dane zasilania (INA219)
                const char* sourceName = "unknown";
                if (inMsg.sourceId == 1) sourceName = "solar";    // ID 1 -> solar
                else if (inMsg.sourceId == 2) sourceName = "battery"; // ID 2 -> battery

                const char* metric = "unknown";
                bool validMetric = true;
                switch(inMsg.type) {
                    case VOLT: metric = "voltage"; break;
                    case CURR: metric = "current"; break;
                    case WATT: metric = "power"; break;
                    default:
                        ESP_LOGE(TAG_PWR, "Unsupported power sensor type: %d (sourceId=%d)", inMsg.type, inMsg.sourceId);
                        validMetric = false;
                        break;
                }
                if (!validMetric) {
                    // Skip publishing for unsupported metric types to avoid invalid MQTT topics
                    continue;
                }
                // Format: home/garden/power/solar/voltage
                snprintf(topicBuffer, 64, "home/garden/power/%s/%s", sourceName, metric);
            }

            snprintf(payloadBuffer, 128, "{\"value\": %.2f, \"ts\": %ld}", inMsg.value, inMsg.timestamp);

            if (client.publish(topicBuffer, payloadBuffer)) {
                // Loguj tylko wybrane, żeby nie śmiecić
                if (inMsg.type == TEMP || inMsg.type == WATT) {
                    ESP_LOGI(TAG_MQTT, "Sent %s: %.2f", topicBuffer, inMsg.value);
                }
            } else {
                ESP_LOGE(TAG_MQTT, "Publish failed!");
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
    esp_log_level_set("*", ESP_LOG_INFO); 
    delay(1000);
    ESP_LOGI(TAG_MAIN, "System Startup - Environment System");

    preferences.begin("smarthome", true);
    String saved_ip = preferences.getString("mqtt_ip", "");
    if (saved_ip.length() > 0) {
        saved_ip.toCharArray(mqtt_server, 40);
        ESP_LOGI(TAG_MAIN, "Loaded MQTT IP: %s", mqtt_server);
    }
    preferences.end();

    WiFi.onEvent(WiFiEvent);
    WiFiManagerParameter custom_mqtt_ip("mqtt_ip", "MQTT Broker IP", mqtt_server, 40);
    wm.addParameter(&custom_mqtt_ip);
    wm.setSaveParamsCallback(saveParamsCallback);
    if(!wm.autoConnect("SmartHome-Env")) {
        ESP.restart();
    }
    
    initTime();

    msgQueue = xQueueCreate(40, sizeof(SensorMeasurement));
    if (msgQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Queue creation failed!");
        while(1); 
    }

    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorControlTask, "SensTask", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }