#pragma once

#define RPI_HOSTNAME "rpi-smarthome"
#define BROKER_USER  "esp32"
#define BROKER_PASS  "esp32"

// ==========================================================
// DYNAMICZNA SELEKCJA TOPICÓW OTA ORAZ NAZW BLE DLA ERM
// ==========================================================
#if defined(MODULE_SECURITY)
    #define OTA_TOPIC       "home/security/update"
    #define PROV_BLE_NAME   "PROV_Security"
    #define MODULE_NAME     "SECURITY SYSTEM"
#elif defined(MODULE_ACCESS)
    #define OTA_TOPIC       "home/access/update"
    #define PROV_BLE_NAME   "PROV_Access"
    #define MODULE_NAME     "ACCESS SYSTEM"
#elif defined(MODULE_ENV)
    #define OTA_TOPIC       "home/environment/update"
    #define PROV_BLE_NAME   "PROV_Environment"
    #define MODULE_NAME     "ENVIRONMENT SYSTEM"
#else
    #define OTA_TOPIC       "home/system/update"
    #define PROV_BLE_NAME   "PROV_default"
    #define MODULE_NAME     "DEFAULT / TEST"
#endif