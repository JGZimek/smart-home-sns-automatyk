#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Inicjalizuje I2C, piny przekaźników, kolejki i uruchamia zadania
void init_environment_system(void);

// Nasłuchuje komend MQTT dotyczących wentylatorów
void environment_mqtt_callback(const char* topic, const char* data, int data_len);

#ifdef __cplusplus
}
#endif