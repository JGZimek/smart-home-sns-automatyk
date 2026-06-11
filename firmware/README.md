# Smart Home – Firmware ESP32

Firmware węzłów końcowych makiety Smart Home. Makieta składa się z **trzech niezależnych płytek ESP32**, które współdzielą **jeden wspólny kod źródłowy**. O roli konkretnej płytki decyduje wybór środowiska kompilacji (flaga `-D MODULE_*`), a nie osobny projekt.

- Framework: **ESP-IDF** (nie Arduino) przez **PlatformIO** (`framework = espidf`).
- Język: C++ (`app_main` w stylu ESP-IDF), współbieżność oparta o **FreeRTOS**.
- Sieć: **Wi-Fi BLE Provisioning** (bez haseł w kodzie) + automatyczne wykrywanie brokera przez **mDNS** + **MQTT** + zdalne **OTA**.

## Architektura kodu

```
firmware/
├─ platformio.ini          # definicje środowisk (= wybór modułu przez -D MODULE_*)
├─ partitions.csv          # układ partycji z dwiema partycjami OTA (ota_0 / ota_1)
├─ include/
│  ├─ module_config.h      # JEDNO miejsce: tożsamość węzła (nazwa, tematy MQTT, BLE)
│  ├─ network_core.h       # wspólny rdzeń: Wi-Fi provisioning, mDNS, OTA
│  ├─ mqtt_core.h          # wspólny rdzeń: klient MQTT + publikator
│  └─ <moduł>_system.h     # interfejs każdego modułu sprzętowego
└─ src/
   ├─ main.cpp             # init NVS -> sieć -> moduł wybrany kompilacyjnie
   ├─ network_core.cpp     # WSPÓLNE dla wszystkich płytek
   ├─ mqtt_core.cpp        # WSPÓLNE dla wszystkich płytek
   ├─ security_system.cpp  # logika modułu (kompilowana tylko gdy MODULE_SECURITY)
   ├─ environment_system.cpp
   └─ access_system.cpp
```

**Wspólna baza vs. moduł.** Pliki `network_core` i `mqtt_core` są identyczne dla każdej płytki i odpowiadają za całość warstwy sieciowej. Każdy plik `*_system.cpp` jest w całości owinięty w `#if defined(MODULE_*)`, więc do obrazu trafia tylko logika wybranego modułu. Dzięki temu zmiana płytki = zmiana środowiska, bez duplikacji kodu sieciowego.

## Środowiska / moduły

| Środowisko PlatformIO | Flaga       | Sprzęt na płytce                                                     | Nazwa BLE        | Temat OTA                |
| :-------------------- | :---------- | :------------------------------------------------------------------ | :--------------- | :----------------------- |
| `security_system`     | `MODULE_SECURITY` | 2× PIR, czujnik płomienia, czujnik gazu (ADC), syrena         | `PROV_Security`  | `home/security/update`   |
| `access_system`       | `MODULE_ACCESS`   | RFID RC522 (SPI), klawiatura 4×4, serwo (zamek)              | `PROV_Access`    | `home/access/update`     |
| `environment_system`  | `MODULE_ENV`      | BME280, TSL2591, 2× INA219 (I2C), 2× przekaźnik wentylatora  | `PROV_Environment` | `home/environment/update` |
| `basic_default`       | `MODULE_DEFAULT`  | brak – profil testowy/deweloperski rdzenia sieciowego        | `PROV_default`   | `home/system/update`     |

`module_config.h` waliduje na etapie kompilacji, że wybrano **dokładnie jeden** moduł.

## Budowanie i wgrywanie

Wymagane: VS Code + rozszerzenie **PlatformIO** (toolchain ESP-IDF pobiera się automatycznie).

```bash
# Build wybranego modułu
pio run -e security_system

# Wgranie przez USB
pio run -e security_system -t upload -t monitor

# Wyczyszczenie zapisanych poświadczeń Wi-Fi (wymusza ponowny provisioning)
pio run -e security_system -t erase
```

W VS Code: dolny pasek PlatformIO → wybór środowiska (`env:...`) → Build / Upload.

## Cykl życia węzła (`main.cpp`)

1. Inicjalizacja NVS (z auto-naprawą przy niezgodnej wersji).
2. `wifi_init_and_prov()` – jeśli brak poświadczeń, startuje BLE provisioning; w przeciwnym razie łączy się z zapisaną siecią. Po **5 nieudanych próbach** automatycznie wraca do BLE provisioningu (fallback).
3. Po uzyskaniu IP startuje `mqtt_discovery_task`, który przez mDNS znajduje `rpi-smarthome` i łączy z brokerem MQTT.
4. Uruchamiana jest logika wybranego modułu (zadania FreeRTOS na rdzeniu 1).

## MQTT – konwencje

Każdy węzeł ma spójną bazę tematów `home/<rodzaj>` (`security`/`access`/`environment`/`system`),
zdefiniowaną w [include/module_config.h](include/module_config.h).

| Temat                        | Kier.          | Opis |
| :--------------------------- | :------------- | :--- |
| `home/<rodzaj>/info`         | pub (retained) | Wizytówka: `{id, kind, name, fw, idf, ip, cmd_topic, diag_topic}` – do auto-wykrywania przez backend. |
| `home/<rodzaj>/availability` | pub (retained) | `ONLINE` / `OFFLINE` (LWT brokera). |
| `home/<rodzaj>/diag`         | pub            | Heartbeat co 30 s: `{id, uptime_s, heap, min_heap, rssi, reset}`. |
| `home/<rodzaj>/cmd`          | sub            | Komendy zarządcze (niżej). |
| `home/<rodzaj>/update`       | sub            | OTA – URL do `firmware.bin` ([OTA_UPDATE.md](OTA_UPDATE.md)). |

**Telemetria / sterowanie sprzętem** – tematy specyficzne dla modułu, np.
`home/security/motion/1`, `home/garden/fan/cooling/set`, `home/access/door/set`.

### Wspólny kanał komend (diagnostyka i rekonfiguracja)

Publikacja na `home/<rodzaj>/cmd` słowa kluczowego lub `{"cmd":"..."}`:

| Komenda      | Działanie |
| :----------- | :-------- |
| `reboot`     | Restart węzła. |
| `reset_wifi` | Kasuje poświadczenia Wi-Fi (NVS) i restartuje do trybu BLE provisioning – zdalna zmiana sieci bez kabla. |
| `identify`   | Ponownie publikuje `info`. |
| `diag`/`ping`| Natychmiast publikuje diagnostykę. |

Przykład (z RPi): `mosquitto_pub -t home/security/cmd -m reboot -u esp32 -P esp32`

Logika wspólna dla wszystkich modułów: [src/device_core.cpp](src/device_core.cpp).

### Odporność / plug-and-play

- **Brak znanej sieci Wi-Fi** → po 5 nieudanych próbach automatyczny powrót do BLE provisioningu.
- **Broker nieosiągalny** (zgaszony lub zmienił IP) → po serii nieudanych prób węzeł ponawia wykrywanie brokera przez mDNS (`mqtt_reconnect_task`).
- **Praca offline** – logika sprzętowa (alarm, klawiatura, RFID, wentylatory) działa lokalnie nawet bez połączenia z brokerem.

## Dokumentacja powiązana

- [HARDWARE.md](HARDWARE.md) – **pinout i instrukcja podłączenia** czujników/aktuatorów do każdej płytki ESP32.
- [MQTT_API.md](MQTT_API.md) – **kontrakt komunikacyjny MQTT dla backendu/frontendu** (wszystkie tematy, payloady, sterowanie).
- [WIFI_PROVISIONING.md](WIFI_PROVISIONING.md) – pierwsza konfiguracja Wi-Fi przez aplikację ESP BLE Provisioning.
- [OTA_UPDATE.md](OTA_UPDATE.md) – zdalna aktualizacja firmware przez MQTT + HTTP.
