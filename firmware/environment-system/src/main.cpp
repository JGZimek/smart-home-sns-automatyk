#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Adafruit_BME280.h>
// #include <ArduinoJson.h> // JSON na razie niepotrzebny, wysyłamy raw values

// --- PIN CONFIGURATION ---
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
// [DISABLED] Other pins for future testing
// #define PIN_SOIL        34
// #define PIN_LIGHT       35
// #define PIN_FAN         19
// #define PIN_SOLAR       32

// --- SETTINGS ---
#define SENSOR_INTERVAL 5000  // Read every 5 seconds
// #define FAN_TEMP_THRESHOLD 28.0

// --- GLOBAL OBJECTS ---
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
Adafruit_BME280 bme;

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
char mqtt_port[6] = "1883";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32"; // Hasło z Twojego Dockera

// --- RTOS: DATA STRUCTURE ---
// Struktura pojedynczej wiadomości w kolejce
struct MqttMessage {
    char topic[64];
    char payload[16];
};

QueueHandle_t msgQueue;

// --- HELPER FUNCTIONS ---

void saveParamsCallback() {
    String ip_str = wm.server->arg("mqtt_ip");
    if (ip_str.length() > 0) {
        ip_str.toCharArray(mqtt_server, 40);
        preferences.begin("smarthome", false);
        preferences.putString("mqtt_ip", mqtt_server);
        preferences.end();
    }
}

void reconnectMqtt() {
    if (!client.connected()) {
        String clientId = "EnvSystem-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            Serial.println("[MQTT] Connected!");
            client.publish("home/garden/system/status", "ONLINE");
        }
    }
}

// ---------------------------------------------------------
// TASK 1: SENSOR & CONTROL (Core 1)
// ---------------------------------------------------------
void sensorControlTask(void * parameter) {
    // Init BME280
    if (!bme.begin(0x76)) {
        Serial.println("[BME] Error! Sensor not found. Check wiring.");
    }
    
    // [DISABLED] Fan Init
    // pinMode(PIN_FAN, OUTPUT);
    // digitalWrite(PIN_FAN, LOW);

    for(;;) {
        // 1. Read Sensors
        float temp = bme.readTemperature();
        float hum = bme.readHumidity();
        float pres = bme.readPressure() / 100.0F;
        
        // [DISABLED] Other sensors
        // int soilRaw = analogRead(PIN_SOIL);
        // int lightRaw = analogRead(PIN_LIGHT);
        
        // 2. [DISABLED] Logic (Fan control)
        /*
        if (temp > FAN_TEMP_THRESHOLD) {
            digitalWrite(PIN_FAN, HIGH);
        } else {
            digitalWrite(PIN_FAN, LOW);
        }
        */

        // 3. Prepare Messages (Individual Topics)
        MqttMessage msg;

        // -- Send Temperature --
        snprintf(msg.topic, sizeof(msg.topic), "home/garden/environment/temperature");
        snprintf(msg.payload, sizeof(msg.payload), "%.2f", temp);
        xQueueSend(msgQueue, &msg, pdMS_TO_TICKS(10));

        // -- Send Humidity --
        snprintf(msg.topic, sizeof(msg.topic), "home/garden/environment/humidity");
        snprintf(msg.payload, sizeof(msg.payload), "%.2f", hum);
        xQueueSend(msgQueue, &msg, pdMS_TO_TICKS(10));

        // -- Send Pressure --
        snprintf(msg.topic, sizeof(msg.topic), "home/garden/environment/pressure");
        snprintf(msg.payload, sizeof(msg.payload), "%.1f", pres);
        xQueueSend(msgQueue, &msg, pdMS_TO_TICKS(10));

        // Wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL));
    }
}

// ---------------------------------------------------------
// TASK 2: NETWORK (Core 0)
// ---------------------------------------------------------
void networkTask(void * parameter) {
    client.setServer(mqtt_server, 1883);
    MqttMessage incomingMsg;

    for(;;) {
        // 1. Maintain WiFi
        if (WiFi.status() != WL_CONNECTED) {
             vTaskDelay(pdMS_TO_TICKS(1000));
             continue;
        }

        // 2. Maintain MQTT
        if (!client.connected()) {
            reconnectMqtt();
        }
        client.loop();

        // 3. Process Queue
        // Odbieramy strukturę z tematem i wartością
        if (xQueueReceive(msgQueue, &incomingMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            client.publish(incomingMsg.topic, incomingMsg.payload);
            
            // Debug logs
            Serial.print("[MQTT] Sent to ");
            Serial.print(incomingMsg.topic);
            Serial.print(": ");
            Serial.println(incomingMsg.payload);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // 1. Load Config
    preferences.begin("smarthome", true);
    String saved_ip = preferences.getString("mqtt_ip", "");
    if (saved_ip.length() > 0) saved_ip.toCharArray(mqtt_server, 40);
    preferences.end();

    // 2. WiFi Manager
    WiFiManagerParameter custom_mqtt_ip("mqtt_ip", "MQTT Broker IP", mqtt_server, 40);
    wm.addParameter(&custom_mqtt_ip);
    wm.setSaveParamsCallback(saveParamsCallback);
    
    // wm.resetSettings(); // Uncomment to force reset WiFi settings for testing
    if(!wm.autoConnect("SmartHome-Env")) {
        ESP.restart();
    }

    // 3. Create Queue
    // Zwiększyłem rozmiar kolejki do 15, bo teraz jeden cykl pomiarowy generuje 3 wiadomości naraz
    msgQueue = xQueueCreate(15, sizeof(MqttMessage));

    // 4. Start Tasks
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorControlTask, "SensTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}