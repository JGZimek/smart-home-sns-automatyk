#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Wspólna dla wszystkich modułów warstwa „tożsamość + diagnostyka + sterowanie".
// Pozwala backendowi automatycznie wykryć węzeł (retained INFO_TOPIC), śledzić
// jego kondycję (DIAG_TOPIC) i zdalnie nim zarządzać (CMD_TOPIC).

// Stabilny identyfikator urządzenia wyprowadzony z adresu MAC, np. "esp32-a1b2c3".
const char* device_id(void);

// Publikuje (retained) INFO_TOPIC z tożsamością węzła – do auto-wykrywania przez backend.
void device_publish_info(void);

// Publikuje natychmiast DIAG_TOPIC: uptime, wolny heap, RSSI, powód resetu.
void device_publish_diag(void);

// Uruchamia zadanie cyklicznie publikujące diagnostykę (heartbeat).
void device_start_diag_task(void);

// Obsługuje komendę z CMD_TOPIC (reboot / reset_wifi / identify / diag).
// Zwraca true, jeśli komendę rozpoznano. Łańcuch musi być zakończony NUL.
bool device_handle_command(const char* data, int data_len);

#ifdef __cplusplus
}
#endif
