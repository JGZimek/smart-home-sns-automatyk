#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include <time.h>
#include "esp_log.h"

// --- TAGI LOGÓW ---
static const char* TAG_MAIN = "ACCESS";
static const char* TAG_RFID = "RFID";
static const char* TAG_KEY  = "KEYPAD";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_TIME = "TIME";

// --- KONFIGURACJA PINÓW ---
// RFID (SPI: SDA=5, SCK=18, MOSI=23, MISO=19)
#define SS_PIN  5
#define RST_PIN 22  // Pin 22 jest bezpieczny (0 powoduje problemy z bootowaniem)
// Servo
#define SERVO_PIN 4 // Pin 4 jest bezpieczny (13 kolidował z klawiaturą)

// Klawiatura (4x4)
byte rowPins[4] = {32, 33, 25, 26}; 
byte colPins[4] = {27, 14, 12, 13}; 

// --- USTAWIENIA CZASU ---
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // UTC+1
const int   daylightOffset_sec = 3600; // UTC+2 (Lato)

// --- OBIEKTY ---
WiFiManager wm;
WiFiClient espClient;
Preferences preferences;
PubSubClient client(espClient);
Servo doorServo;
MFRC522 rfid(SS_PIN, RST_PIN);

// Klawiatura setup
char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32";

// --- ZMIENNE GLOBALNE ---
volatile bool remoteOpenRequest = false; 

// --- STRUKTURY DANYCH ---
enum EventType { CARD_SCANNED, PIN_ENTERED, DOOR_ACTION };

struct AccessEvent {
    EventType type;
    char payload[32]; 
    time_t timestamp;
};

QueueHandle_t accessQueue;

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
        String clientId = "AccessSys-" + String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            ESP_LOGI(TAG_MQTT, "Connected!");
            client.subscribe("home/entry/door/set");
        } else {
            printMqttError(client.state());
        }
    }
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
    String msg = "";
    for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
    
    ESP_LOGI(TAG_MQTT, "Msg: %s -> %s", topic, msg.c_str());

    if (String(topic) == "home/entry/door/set") {
        if (msg == "OPEN") {
            ESP_LOGI(TAG_MAIN, "Remote OPEN request received");
            remoteOpenRequest = true;
        }
    }
}

// ---------------------------------------------------------
// TASK 1: ACCESS CONTROL (Obsługa sprzętu)
// ---------------------------------------------------------
void accessTask(void * parameter) {
    SPI.begin();
    rfid.PCD_Init();
    doorServo.attach(SERVO_PIN);
    doorServo.write(0); // Stan początkowy: Zamknięte

    ESP_LOGI(TAG_MAIN, "Access Hardware Initialized");

    for(;;) {
        time_t now;
        time(&now);
        AccessEvent evt;
        evt.timestamp = now;
        bool openDoor = false; 
        char actionSource[32] = "UNKNOWN";

        // 1. Sprawdzenie żądania zdalnego (MQTT)
        if (remoteOpenRequest) {
            openDoor = true;
            strcpy(actionSource, "REMOTE_MQTT");
            remoteOpenRequest = false;
        }

        // 2. Obsługa RFID
        if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
            String uid = "";
            for (byte i = 0; i < rfid.uid.size; i++) {
                uid += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
                uid += String(rfid.uid.uidByte[i], HEX);
            }
            ESP_LOGI(TAG_RFID, "Card: %s", uid.c_str());
            
            evt.type = CARD_SCANNED;
            uid.toUpperCase();
            uid.toCharArray(evt.payload, 32);
            xQueueSend(accessQueue, &evt, pdMS_TO_TICKS(10));

            // Logika otwarcia (Demo: każda karta otwiera)
            openDoor = true;
            strcpy(actionSource, "RFID");

            rfid.PICC_HaltA();
            rfid.PCD_StopCrypto1();
        }

        // 3. Obsługa Klawiatury
        char key = keypad.getKey();
        if (key) {
            ESP_LOGI(TAG_KEY, "Key: %c", key);
            evt.type = PIN_ENTERED;
            evt.payload[0] = key;
            evt.payload[1] = '\0';
            xQueueSend(accessQueue, &evt, pdMS_TO_TICKS(10));
            
            if (key == '#') {
                openDoor = true;
                strcpy(actionSource, "KEYPAD");
            }
        }

        // 4. Wykonanie akcji (SERWO)
        if (openDoor) {
            ESP_LOGI(TAG_MAIN, "Opening Door via %s...", actionSource);
            
            AccessEvent actionEvt;
            actionEvt.type = DOOR_ACTION;
            actionEvt.timestamp = now;
            snprintf(actionEvt.payload, 32, "OPENED_BY_%s", actionSource);
            xQueueSend(accessQueue, &actionEvt, pdMS_TO_TICKS(10));

            doorServo.write(90); 
            vTaskDelay(pdMS_TO_TICKS(3000)); // Otwórz na 3s
            doorServo.write(0);
            ESP_LOGI(TAG_MAIN, "Door Closed");
        }

        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

// ---------------------------------------------------------
// TASK 2: NETWORK
// ---------------------------------------------------------
void networkTask(void * parameter) {
    ESP_LOGI(TAG_MQTT, "Task started on Core 0");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);
    
    AccessEvent inMsg;
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
        if (xQueueReceive(accessQueue, &inMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch(inMsg.type) {
                case CARD_SCANNED: snprintf(topicBuf, 64, "home/entry/rfid/scan"); break;
                case PIN_ENTERED:  snprintf(topicBuf, 64, "home/entry/keypad/press"); break;
                case DOOR_ACTION:  snprintf(topicBuf, 64, "home/entry/door/status"); break;
                default:           snprintf(topicBuf, 64, "home/entry/log"); break;
            }
            
            snprintf(jsonBuf, 128, "{\"data\": \"%s\", \"ts\": %ld}", inMsg.payload, inMsg.timestamp);
            
            if (client.publish(topicBuf, jsonBuf)) {
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
    ESP_LOGI(TAG_MAIN, "System Startup - Access System");

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
    
    if(!wm.autoConnect("SmartHome-Access")) {
        ESP_LOGE(TAG_WIFI, "Failed to connect. Restarting...");
        ESP.restart();
    }
    
    // 3. Init Time
    initTime();

    // 4. Create Queue
    accessQueue = xQueueCreate(10, sizeof(AccessEvent));
    if (accessQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Failed to create queue! Halt.");
        while(1);
    }
    
    // 5. Start Tasks
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(accessTask, "AccessTask", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }