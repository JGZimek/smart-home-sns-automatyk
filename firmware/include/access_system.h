#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Inicjalizuje SPI, RFID, Klawiaturę i Serwo
void init_access_system(void);

// Nasłuchuje na żądania otwarcia zamka
void access_mqtt_callback(const char* topic, const char* data, int data_len);

#ifdef __cplusplus
}
#endif