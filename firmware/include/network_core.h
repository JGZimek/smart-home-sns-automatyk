#pragma once
#include <stddef.h>
#include "esp_err.h"

void wifi_init_and_prov(void);
esp_err_t resolve_rpi_ip(char *out_ip_uri, size_t max_len);
void ota_update_task(void *pvParameter);