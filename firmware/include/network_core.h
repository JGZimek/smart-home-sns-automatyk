#pragma once
#include <stddef.h>
#include "esp_err.h"

void wifi_init_and_prov(void);
esp_err_t resolve_rpi_ip(char *out_ip_uri, size_t max_len);
void ota_update_task(void *pvParameter);

// Powiadomienie z warstwy MQTT o udanym połączeniu z brokerem (wyłącza watchdog,
// który w przeciwnym razie po BROKER_PROV_TIMEOUT_S wróciłby do BLE provisioningu).
void network_notify_broker_connected(void);

// Zapisuje nowe dane logowania Wi-Fi do NVS (zdalna zmiana sieci komendą set_wifi).
// Po wywołaniu należy zrestartować węzeł, aby czysto połączył się z nową siecią.
void wifi_set_credentials(const char *ssid, const char *pass);