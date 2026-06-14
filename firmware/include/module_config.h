#pragma once

// ==========================================================
// Smart Home – wspólna konfiguracja tożsamości węzła
// ==========================================================
// Makieta składa się z trzech niezależnych płytek ESP32. Wszystkie
// współdzielą ten sam kod rdzenia sieciowego (network_core, mqtt_core,
// device_core), a o roli danej płytki decyduje DOKŁADNIE JEDNA flaga
// kompilacji MODULE_* ustawiana w środowisku PlatformIO (platformio.ini):
//
//   env:security_system     -> -D MODULE_SECURITY
//   env:access_system       -> -D MODULE_ACCESS
//   env:environment_system  -> -D MODULE_ENV
//   env:basic_default       -> -D MODULE_DEFAULT   (profil testowy)
//
// Ten plik centralizuje wszystkie wartości zależne od modułu.

// --- Wersja firmware (raportowana w topicu info, przydatna przy OTA) ---
#define FIRMWARE_VERSION  "2.2.0"

// --- Wspólna infrastruktura sieciowa ---
#define RPI_HOSTNAME      "rpi-smarthome"   // host mDNS brokera (Raspberry Pi)
#define MQTT_BROKER_PORT  1883
#define BROKER_USER       "smarthome"
#define BROKER_PASS       "smarthome"
#define PROV_POP          "smarthome"       // Proof-of-Possession dla BLE provisioningu

// --- Walidacja: dokładnie jeden moduł musi być wybrany ---
#if (defined(MODULE_SECURITY) + defined(MODULE_ACCESS) + defined(MODULE_ENV) + defined(MODULE_DEFAULT)) != 1
#  error "Wybierz DOKLADNIE JEDEN modul (MODULE_SECURITY / MODULE_ACCESS / MODULE_ENV / MODULE_DEFAULT) przez flage -D w platformio.ini"
#endif

// ==========================================================
// TOŻSAMOŚĆ MODUŁU: rodzaj, nazwa, nazwa BLE
// ==========================================================
#if defined(MODULE_SECURITY)
#  define DEVICE_KIND   "security"
#  define MODULE_NAME   "SECURITY SYSTEM"
#  define PROV_BLE_NAME "PROV_Security"
#elif defined(MODULE_ACCESS)
#  define DEVICE_KIND   "access"
#  define MODULE_NAME   "ACCESS SYSTEM"
#  define PROV_BLE_NAME "PROV_Access"
#elif defined(MODULE_ENV)
#  define DEVICE_KIND   "environment"
#  define MODULE_NAME   "ENVIRONMENT SYSTEM"
#  define PROV_BLE_NAME "PROV_Environment"
#else // MODULE_DEFAULT
#  define DEVICE_KIND   "system"
#  define MODULE_NAME   "DEFAULT / TEST"
#  define PROV_BLE_NAME "PROV_default"
#endif

// ==========================================================
// SPÓJNA PRZESTRZEŃ TEMATÓW MQTT
// ==========================================================
// Bazą jest "home/<rodzaj>". Backend może w pełni sterować i obserwować
// węzeł znając wyłącznie jego rodzaj (auto-wykrycie przez retained INFO_TOPIC).
#define DEVICE_BASE_TOPIC   "home/" DEVICE_KIND

#define OTA_TOPIC           DEVICE_BASE_TOPIC "/update"        // (sub)  URL firmware.bin
#define AVAILABILITY_TOPIC  DEVICE_BASE_TOPIC "/availability"  // (pub, retained) ONLINE / OFFLINE(LWT)
#define INFO_TOPIC          DEVICE_BASE_TOPIC "/info"          // (pub, retained) JSON z tożsamością + capabilities
#define DIAG_TOPIC          DEVICE_BASE_TOPIC "/diag"          // (pub) JSON ze stanem zdrowia węzła
#define CMD_TOPIC           DEVICE_BASE_TOPIC "/cmd"           // (sub)  komendy: reboot / reset_wifi / identify / diag
