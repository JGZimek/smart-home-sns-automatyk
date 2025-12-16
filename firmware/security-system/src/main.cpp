#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <time.h>
#include "esp_log.h"

// --- TAGI LOGÓW ---
static const char* TAG_MAIN = "SECURITY";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_SEC  = "SEC_LOGIC";
static const char* TAG_TIME = "TIME";

// --- KONFIGURACJA PINÓW ---
#define PIN_PIR_1    14
#define PIN_PIR_2    27
#define PIN_FLAME    26  // Często czujniki płomienia dają LOW przy wykryciu
#define PIN_GAS      34  // Analogowy (MQ-2/MQ-7 etc.)
#define PIN_SIREN    25  // Wyjście na tranzystor/przekaźnik syreny

// --- USTAWIENIA ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // UTC+1
const int   daylightOffset_sec = 3600; // UTC+2 (Lato)
#define GAS_THRESHOLD 2000             // Próg alarmowy dla gazu (0-4095)

// --- OBIEKTY ---
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
char mqtt_port[6] = "1883";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32";

// --- ZMIENNE GLOBALNE ---
volatile bool systemArmed = false; // Czy alarm jest uzbrojony?

// --- STRUKTURY DANYCH ---
enum AlarmType { MOTION, FIRE, GAS, STATUS_CHANGE };

struct SecurityEvent {
    AlarmType type;
    int sensorId; // Np. 1 dla PIR1, 2 dla PIR2
    int value;    // 0/1 lub wartość analogowa
    time_t timestamp;
};

QueueHandle_t secQueue;

// --- FUNKCJE POMOCNICZE ---

void saveParamsCallback() {
    ESP_LOGI(TAG_WIFI, "Saving configuration...");
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
            ESP_LOGI(TAG_WIFI, "Connected! IP: %s", WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            ESP_LOGW(TAG_WIFI, "Disconnected! Reconnecting...");
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
        ESP_LOGI(TAG_TIME, "Time Synced: %s", timeStr);
    }
}

// --- MQTT LOGIC ---

void reconnectMqtt() {
    if (!client.connected()) {
        ESP_LOGI(TAG_MQTT, "Connecting to broker at %s...", mqtt_server);
        String clientId = "SecSys-" + String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            ESP_LOGI(TAG_MQTT, "Connected!");
            // WAŻNE: Subskrypcja kanału uzbrajania
            client.subscribe("home/security/arm/set");
        } else {
            printMqttError(client.state());
        }
    }
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
    String msg = "";
    for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
    
    ESP_LOGI(TAG_MQTT, "Cmd: %s -> %s", topic, msg.c_str());

    if (String(topic) == "home/security/arm/set") {
        SecurityEvent evt;
        time(&evt.timestamp);
        evt.type = STATUS_CHANGE;
        evt.sensorId = 0;

        if (msg == "ARM") {
            systemArmed = true;
            evt.value = 1;
            ESP_LOGW(TAG_SEC, "SYSTEM ARMED!");
        } else if (msg == "DISARM") {
            systemArmed = false;
            evt.value = 0;
            digitalWrite(PIN_SIREN, LOW); // Wyłącz syrenę przy rozbrojeniu
            ESP_LOGW(TAG_SEC, "SYSTEM DISARMED");
        }
        // Wyślij status zwrotny do kolejki
        xQueueSend(secQueue, &evt, pdMS_TO_TICKS(10));
    }
}

// ---------------------------------------------------------
// TASK 1: SECURITY MONITORING (Obsługa czujników)
// ---------------------------------------------------------
void securityTask(void * parameter) {
    pinMode(PIN_PIR_1, INPUT);
    pinMode(PIN_PIR_2, INPUT);
    pinMode(PIN_FLAME, INPUT); 
    pinMode(PIN_GAS, INPUT); // Analog input config
    pinMode(PIN_SIREN, OUTPUT);
    digitalWrite(PIN_SIREN, LOW);

    ESP_LOGI(TAG_MAIN, "Sensors initialized");

    for(;;) {
        time_t now;
        time(&now);
        SecurityEvent evt;
        evt.timestamp = now;

        // 1. ODCZYT PIR (Tylko jeśli uzbrojony)
        if (systemArmed) {
            // PIR 1
            if (digitalRead(PIN_PIR_1) == HIGH) {
                ESP_LOGW(TAG_SEC, "Motion Detected PIR 1");
                evt.type = MOTION; evt.sensorId = 1; evt.value = 1;
                xQueueSend(secQueue, &evt, pdMS_TO_TICKS(10));
                
                digitalWrite(PIN_SIREN, HIGH); 
                vTaskDelay(pdMS_TO_TICKS(5000)); // Alarm przez 5s
            }
            // PIR 2
            if (digitalRead(PIN_PIR_2) == HIGH) {
                ESP_LOGW(TAG_SEC, "Motion Detected PIR 2");
                evt.type = MOTION; evt.sensorId = 2; evt.value = 1;
                xQueueSend(secQueue, &evt, pdMS_TO_TICKS(10));
                
                digitalWrite(PIN_SIREN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }

        // 2. ODCZYT OGNIOWY (Zawsze aktywny!)
        // Uwaga: Większość modułów płomienia daje LOW przy wykryciu ognia
        if (digitalRead(PIN_FLAME) == LOW) { 
            ESP_LOGE(TAG_SEC, "FIRE DETECTED!");
            evt.type = FIRE; evt.sensorId = 1; evt.value = 1;
            xQueueSend(secQueue, &evt, pdMS_TO_TICKS(10));
            digitalWrite(PIN_SIREN, HIGH); 
        }

        // 3. ODCZYT GAZU (Zawsze aktywny!)
        int gasValue = analogRead(PIN_GAS);
        if (gasValue > GAS_THRESHOLD) {
            ESP_LOGE(TAG_SEC, "GAS LEAK! Level: %d", gasValue);
            evt.type = GAS; evt.sensorId = 1; evt.value = gasValue;
            xQueueSend(secQueue, &evt, pdMS_TO_TICKS(10));
            digitalWrite(PIN_SIREN, HIGH);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Częstotliwość sprawdzania (5Hz)
    }
}

// ---------------------------------------------------------
// TASK 2: NETWORK
// ---------------------------------------------------------
void networkTask(void * parameter) {
    ESP_LOGI(TAG_MQTT, "Task started on Core 0");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);

    SecurityEvent inMsg;
    char topicBuf[64];
    char jsonBuf[128];

    for(;;) {
        // 1. WiFi Check
        if (WiFi.status() != WL_CONNECTED) {
             vTaskDelay(pdMS_TO_TICKS(2000));
             continue;
        }

        // 2. MQTT Check
        if (!client.connected()) {
            reconnectMqtt();
            if (!client.connected()) {
                 vTaskDelay(pdMS_TO_TICKS(5000)); 
                 continue;
            }
        }
        client.loop();

        // 3. Queue Processing
        if (xQueueReceive(secQueue, &inMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch(inMsg.type) {
                case MOTION: snprintf(topicBuf, 64, "home/security/motion/%d", inMsg.sensorId); break;
                case FIRE:   snprintf(topicBuf, 64, "home/security/fire"); break;
                case GAS:    snprintf(topicBuf, 64, "home/security/gas"); break;
                case STATUS_CHANGE: snprintf(topicBuf, 64, "home/security/status"); break;
                default:     snprintf(topicBuf, 64, "home/security/log"); break;
            }
            
            snprintf(jsonBuf, 128, "{\"val\": %d, \"ts\": %ld}", inMsg.value, inMsg.timestamp);
            
            if(client.publish(topicBuf, jsonBuf)) {
                ESP_LOGI(TAG_MQTT, "Sent: %s", jsonBuf);
            } else {
                ESP_LOGE(TAG_MQTT, "Publish failed!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_INFO);
    delay(1000);
    ESP_LOGI(TAG_MAIN, "System Startup - Security System");

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
    WiFi.onEvent(WiFiEvent);
    WiFiManagerParameter custom_mqtt_ip("mqtt_ip", "MQTT Broker IP", mqtt_server, 40);
    wm.addParameter(&custom_mqtt_ip);
    wm.setSaveParamsCallback(saveParamsCallback);
    
    if(!wm.autoConnect("SmartHome-Security")) {
        ESP_LOGE(TAG_WIFI, "Failed to connect. Restarting...");
        ESP.restart();
    }
    
    // 3. Init Time
    initTime();

    // 4. Create Queue
    secQueue = xQueueCreate(20, sizeof(SecurityEvent));
    if (secQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create queue! Halt.");
        while(1);
    }
    
    // 5. Start Tasks
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(securityTask, "SecTask", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }