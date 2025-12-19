#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_TSL2591.h>
#include <time.h>
#include "esp_log.h"

// --- LOG TAGS ---
static const char* TAG_MAIN = "ENV_SYS";
static const char* TAG_WIFI = "WIFI";
static const char* TAG_MQTT = "MQTT";
static const char* TAG_SENS = "SENS";
static const char* TAG_PWR  = "POWER";
static const char* TAG_CTRL = "CTRL";
static const char* TAG_TIME = "TIME";

// --- PIN CONFIG ---
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
// Fan Control Pins (Relay Module is usually Active LOW)
#define PIN_FAN_COOLING 18 // Fan 1 -> Cooling
#define PIN_FAN_VENT    19 // Fan 2 -> Ventilation

// --- I2C ADDRESSES ---
#define INA1_ADDR       0x45 // Wattmeter 1 (Solar)
#define INA2_ADDR       0x44 // Wattmeter 2 (Battery)
// TSL2591 default address is 0x29 (Fixed)

// --- SETTINGS ---
#define SENSOR_INTERVAL 5000
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // UTC+1
const int   daylightOffset_sec = 3600; // UTC+2 (Summer)

// --- GLOBAL OBJECTS ---
WiFiManager wm;
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// Sensors
Adafruit_BME280 bme;
Adafruit_INA219 pwr1(INA1_ADDR);
Adafruit_INA219 pwr2(INA2_ADDR);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// Sensor status flags
bool bme_connected = false;
bool pwr1_connected = false;
bool pwr2_connected = false;
bool tsl_connected = false;

// --- MQTT CONFIG ---
char mqtt_server[40] = "192.168.1.57";
char mqtt_port[6] = "1883";
const char* mqtt_user = "esp32";
const char* mqtt_pass = "esp32";

// --- DATA STRUCTURES ---
enum SensorType { TEMP, HUM, PRES, VOLT, CURR, WATT, LIGHT };

// Added MSG_ERROR type to handle errors via queue instead of direct publish from callback
enum MsgType { SENSOR_DATA, FAN_STATUS, ERROR_MSG }; 

struct SensorMeasurement {
    SensorType type;
    uint8_t sourceId; // 0=Env, 1=Solar, 2=Battery
    float value;
    time_t timestamp;
};

// Command structure for fans
// fanId: 1 = Cooling, 2 = Ventilation
struct FanCommand {
    uint8_t fanId; 
    bool state;    // true=ON, false=OFF
};

// Structure for Status Update (Feedback to backend)
struct FanStatus {
    uint8_t fanId;
    bool state;
};

// Unified structure to handle errors safely in network task
struct NetworkMessage {
    MsgType msgType;
    union {
        SensorMeasurement sensor;
        FanStatus fan;
        char errorText[32];
    } data;
};

QueueHandle_t msgQueue; // Sensor data queue (now holds NetworkMessage)
QueueHandle_t cmdQueue; // Control command queue

// --- HELPER FUNCTIONS ---

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

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Static buffer to avoid dynamic allocation
    char msg[64];
    if (length >= sizeof(msg)) length = sizeof(msg) - 1;
    memcpy(msg, payload, length);
    msg[length] = '\0';
    
    ESP_LOGI(TAG_MQTT, "CMD received: %s -> %s", topic, msg);

    FanCommand cmd;
    bool validCommand = false;

    // Parse Topic - Using meaningful names
    if (strcmp(topic, "home/garden/fan/cooling/set") == 0) {
        cmd.fanId = 1; // Cooling
        validCommand = true;
    } else if (strcmp(topic, "home/garden/fan/vent/set") == 0) {
        cmd.fanId = 2; // Ventilation
        validCommand = true;
    }

    // Parse Payload
    if (validCommand) {
        if (msg.equalsIgnoreCase("ON")) cmd.state = true;
        else if (msg.equalsIgnoreCase("OFF")) cmd.state = false;
        else {
            ESP_LOGW(TAG_MQTT, "Invalid payload: %s", msg);
            validCommand = false;
        }
    }

    // Send to Control Task if valid
    if (validCommand) {
        if (xQueueSend(cmdQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG_MQTT, "Command Queue Full! Dropping command.");
            // Inform MQTT sender that the command was rejected due to overload
            client.publish("home/garden/system/error", "CMD_REJECTED_QUEUE_FULL", false);
        }
    }
}

void reconnectMqtt() {
    if (!client.connected()) {
        ESP_LOGI(TAG_MQTT, "Connecting to broker at %s...", mqtt_server);
        String clientId = "EnvSystem-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            ESP_LOGI(TAG_MQTT, "Connected successfully!");
            client.publish("home/garden/system/status", "ONLINE");
            
            // Subscribe to control topics (Meaningful names)
            bool coolingSubOk = client.subscribe("home/garden/fan/cooling/set");
            bool ventSubOk    = client.subscribe("home/garden/fan/vent/set");
            
            if (coolingSubOk && ventSubOk) {
                ESP_LOGI(TAG_MQTT, "Subscribed to fan control topics");
            } else {
                ESP_LOGE(TAG_MQTT, "Failed to subscribe to fan control topics. Retrying connection...");
                client.disconnect();
            }
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

void configureTSL() {
    // MEDIUM Gain (25x) - Good balance for indoor/outdoor
    tsl.setGain(TSL2591_GAIN_MED);
    // 100ms integration time - Fast enough response
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}

// ---------------------------------------------------------
// TASK 3: CONTROL TASK (Core 1)
// ---------------------------------------------------------
// Handles Fan Relay Logic (Active LOW: LOW=ON, HIGH=OFF)
void controlTask(void * parameter) {
    ESP_LOGI(TAG_CTRL, "Task started");
    
    // Initialize Pins
    pinMode(PIN_FAN_COOLING, OUTPUT);
    pinMode(PIN_FAN_VENT, OUTPUT);
    
    // Set initial state to OFF (HIGH for Active Low relays)
    digitalWrite(PIN_FAN_COOLING, HIGH);
    digitalWrite(PIN_FAN_VENT, HIGH);

    FanCommand cmd;

    for(;;) {
        if (xQueueReceive(cmdQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            // Validate fanId
            if (cmd.fanId != 1 && cmd.fanId != 2) {
                ESP_LOGE(TAG_CTRL, "Received invalid fanId: %d", cmd.fanId);
                continue;
            }

            int pin = (cmd.fanId == 1) ? PIN_FAN_COOLING : PIN_FAN_VENT;
            
            // LOGIC INVERSION for Relay Module (Active LOW)
            // ON command -> LOW signal
            // OFF command -> HIGH signal
            int level = (cmd.state) ? LOW : HIGH; 
            
            digitalWrite(pin, level);
            
            const char* fanName = (cmd.fanId == 1) ? "Cooling" : "Ventilation";
            const char* stateStr = (cmd.state) ? "ON" : "OFF";
            
            ESP_LOGI(TAG_CTRL, "Fan %s set to %s (Pin Level: %s)", fanName, stateStr, (level==LOW)?"LOW":"HIGH");
            
            // Send feedback status to MQTT task
            NetworkMessage netMsg;
            netMsg.msgType = FAN_STATUS;
            netMsg.data.fan.fanId = cmd.fanId;
            netMsg.data.fan.state = cmd.state;

            if (xQueueSend(msgQueue, &netMsg, pdMS_TO_TICKS(10)) != pdTRUE) {
                ESP_LOGE(TAG_CTRL, "Failed to enqueue fan status feedback");
            }
        }
        
        // Task maintenance logic can go here (e.g. stack monitoring)
    }
}

// ---------------------------------------------------------
// TASK 1: SENSOR & CONTROL (Core 1)
// ---------------------------------------------------------
void sensorControlTask(void * parameter) {
    ESP_LOGI(TAG_SENS, "Task started on Core 1");

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        ESP_LOGE(TAG_SENS, "Failed to initialize I2C bus on SDA=%d, SCL=%d", PIN_I2C_SDA, PIN_I2C_SCL);
        bme_connected  = false;
        tsl_connected  = false;
        pwr1_connected = false;
        pwr2_connected = false;
    } else {
        ESP_LOGI(TAG_SENS, "I2C bus initialized on SDA=%d, SCL=%d", PIN_I2C_SDA, PIN_I2C_SCL);

        // 1. Init BME280
        if (!bme.begin(0x76, &Wire)) {
            ESP_LOGE(TAG_SENS, "BME280 not found!");
            bme_connected = false;
        } else {
            ESP_LOGI(TAG_SENS, "BME280 initialized OK.");
            bme_connected = true;
        }

        // 2. Init TSL2591
        if (!tsl.begin(&Wire)) {
            ESP_LOGE(TAG_SENS, "TSL2591 Not Found!");
            tsl_connected = false;
        } else {
            ESP_LOGI(TAG_SENS, "TSL2591 initialized OK.");
            tsl_connected = true;
            configureTSL();
        }

        // 3. Init Wattmeter 1 (Solar)
        if (!pwr1.begin(&Wire)) {
            ESP_LOGE(TAG_PWR, "INA219 #1 (Addr 0x%X) Not Found", INA1_ADDR);
            pwr1_connected = false;
        } else {
            pwr1.setCalibration_32V_2A(); 
            pwr1_connected = true;
            ESP_LOGI(TAG_PWR, "INA219 #1 (Solar) Connected");
        }

        // 4. Init Wattmeter 2 (Battery)
        if (!pwr2.begin(&Wire)) {
            ESP_LOGE(TAG_PWR, "INA219 #2 (Addr 0x%X) Not Found", INA2_ADDR);
            pwr2_connected = false;
        } else {
            pwr2.setCalibration_32V_2A(); 
            pwr2_connected = true;
            ESP_LOGI(TAG_PWR, "INA219 #2 (Battery) Connected");
        }
    }
    
    for(;;) {
        time_t now;
        time(&now); 

        // --- BME280 Readings ---
        if (bme_connected) {
            float temp = bme.readTemperature();
            float hum = bme.readHumidity();
            float pres = bme.readPressure() / 100.0F;

            if (isnan(temp) || isnan(hum) || isnan(pres)) {
                ESP_LOGW(TAG_SENS, "Invalid BME280 reading (NaN). Skipping.");
            } else {
                NetworkMessage m;
                m.msgType = SENSOR_DATA;
                m.data.sensor.timestamp = now;
                m.data.sensor.sourceId = 0; // ID 0 = Environment

                m.data.sensor.type = TEMP; m.data.sensor.value = temp;
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env TEMP");
                
                m.data.sensor.type = HUM; m.data.sensor.value = hum; 
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env HUM");

                m.data.sensor.type = PRES; m.data.sensor.value = pres; 
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env PRES");
            }
        }

        // --- TSL2591 Readings ---
        if (tsl_connected) {
            uint32_t lum = tsl.getFullLuminosity();
            uint16_t ir = lum >> 16;
            uint16_t full = lum & 0xFFFF;

            if (full == 0xFFFF) {
                ESP_LOGW(TAG_SENS, "TSL2591 Saturated! (Too bright)");
            } else {
                float lux = tsl.calculateLux(full, ir);
                if (!isnan(lux)) {
                    NetworkMessage m;
                    m.msgType = SENSOR_DATA;
                    m.data.sensor.timestamp = now;
                    m.data.sensor.sourceId = 0; // Environment group
                    m.data.sensor.type = LIGHT; 
                    m.data.sensor.value = lux;
                    if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_SENS, "Q Full: Env LIGHT");
                }
            }
        }

        // --- Wattmeter 1 (Solar) ---
        if (pwr1_connected) {
            float v = pwr1.getBusVoltage_V();
            float c_mA = pwr1.getCurrent_mA();
            float w_mW = pwr1.getPower_mW();

            if (isnan(v) || isnan(c_mA) || isnan(w_mW)) {
                ESP_LOGW(TAG_PWR, "Invalid INA219 #1 (Solar) reading. Skipping.");
            } else {
                NetworkMessage m;
                m.msgType = SENSOR_DATA;
                m.data.sensor.timestamp = now;
                m.data.sensor.sourceId = 1;
                
                m.data.sensor.type = VOLT; m.data.sensor.value = v; 
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar VOLT");

                m.data.sensor.type = CURR; m.data.sensor.value = c_mA / 1000.0f; // mA -> A
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar CURR");

                m.data.sensor.type = WATT; m.data.sensor.value = w_mW / 1000.0f; // mW -> W
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Solar WATT");
            }
        }

        // --- Wattmeter 2 (Battery) ---
        if (pwr2_connected) {
            float v = pwr2.getBusVoltage_V();
            float c_mA = pwr2.getCurrent_mA();
            float w_mW = pwr2.getPower_mW();

            if (isnan(v) || isnan(c_mA) || isnan(w_mW)) {
                ESP_LOGW(TAG_PWR, "Invalid INA219 #2 (Battery) reading. Skipping.");
            } else {
                NetworkMessage m;
                m.msgType = SENSOR_DATA;
                m.data.sensor.timestamp = now;
                m.data.sensor.sourceId = 2;
                
                m.data.sensor.type = VOLT; m.data.sensor.value = v; 
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt VOLT");

                m.data.sensor.type = CURR; m.data.sensor.value = c_mA / 1000.0f; // mA -> A
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt CURR");

                m.data.sensor.type = WATT; m.data.sensor.value = w_mW / 1000.0f; // mW -> W
                if(xQueueSend(msgQueue, &m, pdMS_TO_TICKS(10)) != pdTRUE) ESP_LOGW(TAG_PWR, "Q Full: Batt WATT");
            }
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
    client.setCallback(mqttCallback); 

    NetworkMessage inMsg;
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

        // Process Incoming Messages (Sensors, Fan Status, Errors)
        if (xQueueReceive(msgQueue, &inMsg, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            if (inMsg.msgType == SENSOR_DATA) {
                // --- TOPIC GENERATION ---
                if (inMsg.data.sensor.sourceId == 0) {
                    switch(inMsg.data.sensor.type) {
                        case TEMP: snprintf(topicBuffer, 64, "home/garden/env/temperature"); break;
                        case HUM:  snprintf(topicBuffer, 64, "home/garden/env/humidity"); break;
                        case PRES: snprintf(topicBuffer, 64, "home/garden/env/pressure"); break;
                        case LIGHT: snprintf(topicBuffer, 64, "home/garden/env/light"); break;
                        default:   snprintf(topicBuffer, 64, "home/garden/env/log"); break;
                    }
                } else {
                    const char* sourceName = "unknown";
                    if (inMsg.data.sensor.sourceId == 1) sourceName = "solar";
                    else if (inMsg.data.sensor.sourceId == 2) sourceName = "battery";

                    const char* metric = "unknown";
                    bool validMetric = true;
                    switch(inMsg.data.sensor.type) {
                        case VOLT: metric = "voltage"; break;
                        case CURR: metric = "current"; break;
                        case WATT: metric = "power"; break;
                        default:
                            ESP_LOGE(TAG_PWR, "Unsupported type: %d", inMsg.data.sensor.type);
                            validMetric = false;
                            break;
                    }
                    
                    if (validMetric) {
                        snprintf(topicBuffer, 64, "home/garden/power/%s/%s", sourceName, metric);
                    } else {
                        snprintf(topicBuffer, 64, "home/garden/power/%s/error", sourceName);
                    }
                }

                // Payload formatting
                if (inMsg.data.sensor.type == WATT || 
                    inMsg.data.sensor.type == CURR || 
                    inMsg.data.sensor.type == LIGHT) 
                {
                    snprintf(payloadBuffer, 128, "{\"value\": %.4f, \"ts\": %ld}", 
                             inMsg.data.sensor.value, 
                             inMsg.data.sensor.timestamp);
                } else {
                    snprintf(payloadBuffer, 128, "{\"value\": %.2f, \"ts\": %ld}", 
                             inMsg.data.sensor.value, 
                             inMsg.data.sensor.timestamp);
                }

                if (client.publish(topicBuffer, payloadBuffer)) {
                    if (inMsg.data.sensor.type == LIGHT) ESP_LOGI(TAG_MQTT, "Sent Light: %.1f Lux", inMsg.data.sensor.value);
                }
            } 
            else if (inMsg.msgType == FAN_STATUS) {
                // Process Fan Status Feedback
                const char* fanName = (inMsg.data.fan.fanId == 1) ? "cooling" : "vent";
                snprintf(topicBuffer, 64, "home/garden/fan/%s/state", fanName);
                const char* statePayload = (inMsg.data.fan.state) ? "ON" : "OFF";
                
                if (client.publish(topicBuffer, statePayload)) {
                    ESP_LOGI(TAG_MQTT, "Sent State: %s -> %s", topicBuffer, statePayload);
                } else {
                    ESP_LOGE(TAG_MQTT, "Failed to publish State: %s -> %s", topicBuffer, statePayload);
                }
            }
            else if (inMsg.msgType == ERROR_MSG) {
                // Process Error Messages
                client.publish("home/garden/system/error", inMsg.data.errorText);
                ESP_LOGE(TAG_MQTT, "Published Error: %s", inMsg.data.errorText);
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

    // Queues
    msgQueue = xQueueCreate(100, sizeof(NetworkMessage));
    cmdQueue = xQueueCreate(20, sizeof(FanCommand)); 
    statusQueue = xQueueCreate(20, sizeof(FanStatus));

    if (msgQueue == NULL || cmdQueue == NULL) {
        ESP_LOGE(TAG_MAIN, "Queue creation failed!");
        while(1); 
    }

    // Start Tasks
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorControlTask, "SensTask", 6144, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(controlTask, "CtrlTask", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }