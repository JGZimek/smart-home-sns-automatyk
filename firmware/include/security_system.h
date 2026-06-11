#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Inicjalizuje piny, ADC i uruchamia zadanie FreeRTOS dla systemu bezpieczeństwa
void init_security_system(void);

// Funkcja zwrotna wywoływana przez rdzeń MQTT po otrzymaniu wiadomości
void security_mqtt_callback(const char* topic, const char* data, int data_len);

#ifdef __cplusplus
}
#endif