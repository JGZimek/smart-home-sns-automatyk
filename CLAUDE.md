# CLAUDE.md

Wskazówki dla Claude Code (i innych agentów) pracujących w tym repozytorium.

## Cel projektu

Makieta **Smart Home** sterowana przez **trzy niezależne płytki ESP32**, spięte przez lokalny broker MQTT (Mosquitto) na Raspberry Pi. Każda płytka pełni inną rolę (bezpieczeństwo / dostęp / środowisko), ale wszystkie współdzielą jeden wspólny kod firmware. Język interfejsu i komentarzy w projekcie: **polski**.

### Mapa dokumentacji (utrzymuj spójność przy zmianach)
- [README.md](README.md) – nadrzędny hub projektu (architektura, nawigacja wg roli, status).
- [GETTING_STARTED.md](GETTING_STARTED.md) – uruchomienie całości od zera (dla osoby z gotowym sprzętem).
- [firmware/README.md](firmware/README.md) – architektura i rozwój firmware.
- [firmware/HARDWARE.md](firmware/HARDWARE.md) – pinout / podłączenie (dla montażysty).
- [firmware/MQTT_API.md](firmware/MQTT_API.md) – kontrakt MQTT (dla backendu).
- [firmware/WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md), [firmware/OTA_UPDATE.md](firmware/OTA_UPDATE.md) – Wi-Fi BLE i OTA.
- [scripts/README.md](scripts/README.md) – serwer RPi (patrz uwaga niżej).

> Zmieniając tematy MQTT / piny / profile, zaktualizuj odpowiednio MQTT_API.md / HARDWARE.md / README. Te pliki mają być źródłem prawdy dla osób spoza firmware.

## Układ repozytorium (monorepo)

| Katalog        | Zawartość |
| :------------- | :-------- |
| `firmware/`    | Firmware ESP32 (ESP-IDF + PlatformIO). **Główna część projektu.** |
| `scripts/`     | Plug-and-play serwer Raspberry Pi (Bash + Python) na Ubuntu Server: instalator one-liner (`bootstrap.sh`→`setup.sh`/`setup.d/`), broker, mDNS, CLI `smarthome`, monitor węzłów + dashboard, awaryjny Wi-Fi AP, LCD, Tailscale, watchdogi. Konfiguracja runtime w `/etc/smarthome/smarthome.env`. |
| `mosquitto/`   | Konfiguracja brokera MQTT (uruchamiany przez `docker-compose.yml`). |
| `backend/`     | Placeholder (tylko `.gitkeep`) – jeszcze nie zaimplementowany. |
| `frontend/`    | Placeholder (tylko `.gitkeep`) – jeszcze nie zaimplementowany. |

Broker uruchamia się z katalogu głównego: `docker compose up -d` (porty 1883 MQTT, 9001 WS).

## Firmware – kluczowa architektura

Pełny opis: [firmware/README.md](firmware/README.md). Najważniejsze zasady:

- **Jeden kod, trzy płytki.** Rola płytki = wybór środowiska PlatformIO, które ustawia flagę `-D MODULE_*`. Nie ma osobnych projektów per płytka.
- **Wspólna baza sieciowa:** `firmware/src/network_core.cpp` (Wi-Fi BLE provisioning, mDNS, OTA) oraz `firmware/src/mqtt_core.cpp` (klient MQTT, publikator, routing wiadomości). Ten kod jest identyczny dla każdej płytki — zmiany tutaj dotyczą wszystkich modułów.
- **Logika modułów:** `firmware/src/{security,environment,access}_system.cpp`. Każdy plik jest w całości owinięty w `#if defined(MODULE_*)`, więc kompiluje się tylko dla swojego środowiska.
- **Tożsamość węzła scentralizowana** w `firmware/include/module_config.h`: nazwa modułu, `OTA_TOPIC`, `AVAILABILITY_TOPIC`, `PROV_BLE_NAME`, dane brokera. Tu też jest walidacja „dokładnie jeden moduł”. **Dodając nowy moduł, zaczynaj od tego pliku** + `platformio.ini` + `idf_component.yml` (zależności sprzętowe) + nowy `*_system.cpp/.h`.

Środowiska: `basic_default` (test rdzenia), `security_system`, `access_system`, `environment_system`. Domyślne: `basic_default`.

**Pinout / podłączenie sprzętu:** [firmware/HARDWARE.md](firmware/HARDWARE.md) (które czujniki/aktuatory do których GPIO na każdej płytce). Źródłem prawdy są `#define`-y pinów w `src/*_system.cpp` — zmieniając piny, aktualizuj HARDWARE.md.

## Budowanie / wgrywanie firmware

```bash
cd firmware
pio run -e security_system                 # build
pio run -e security_system -t upload -t monitor
pio run -e security_system -t erase        # czyści poświadczenia Wi-Fi (NVS)
```

`managed_components/` (np. `espressif__mdns`) są pobierane przez ESP-IDF Component Manager z `firmware/src/idf_component.yml` i **nie są wersjonowane** (gitignore). Nie edytować ręcznie.

> Środowisko deweloperskie to Windows + PowerShell. Toolchain ESP-IDF zwykle nie jest dostępny w sesji agenta — zmian w firmware **nie da się tu skompilować**; rób edycje precyzyjnie i trzymaj się API już używanych w danym pliku.

## Konwencje MQTT

Spójna przestrzeń tematów: bazą każdego węzła jest `home/<rodzaj>` (`security` /
`access` / `environment` / `system`) – definicje w `module_config.h`.

| Temat                        | Kier. | Opis |
| :--------------------------- | :---- | :--- |
| `home/<rodzaj>/info`         | pub (retained) | „Wizytówka" węzła: id (z MAC), rodzaj, nazwa, wersja fw, IP, topic cmd/diag. **Backend auto-wykrywa urządzenia czytając te retained wiadomości.** |
| `home/<rodzaj>/availability` | pub (retained) | `ONLINE` / `OFFLINE` (LWT brokera przy rozłączeniu). |
| `home/<rodzaj>/diag`         | pub   | Heartbeat co 30 s: uptime, wolny heap, RSSI, powód resetu. |
| `home/<rodzaj>/cmd`          | sub   | Komendy zarządcze (patrz niżej). |
| `home/<rodzaj>/update`       | sub   | OTA: URL do `firmware.bin`. Szczegóły: [firmware/OTA_UPDATE.md](firmware/OTA_UPDATE.md). |

Telemetria i sterowanie sprzętem są specyficzne dla modułu (np. `home/security/motion/1`,
`home/garden/fan/cooling/set`, `home/access/door/set`).

**Wspólny kanał komend (`home/<rodzaj>/cmd`)** – przyjmuje słowo kluczowe lub
JSON `{"cmd":"..."}`:
- `reboot` – restart węzła,
- `reset_wifi` – kasuje poświadczenia Wi-Fi i restartuje do BLE provisioningu (zdalna rekonfiguracja sieci, np. po przewiezieniu makiety),
- `identify` – ponownie publikuje `info`,
- `diag` / `ping` – natychmiastowa publikacja diagnostyki.

Implementacja tych funkcji jest wspólna: `firmware/src/device_core.cpp` (+ `device_core.h`).

**Pełny kontrakt MQTT dla backendu** (wszystkie tematy per moduł, payloady, QoS/retained, przykłady): [firmware/MQTT_API.md](firmware/MQTT_API.md).

- **Provisioning:** BLE, PoP = `smarthome` (stała `PROV_POP`). Szczegóły: [firmware/WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md).
- **Plug-and-play / odporność:** brak Wi-Fi → po 5 próbach automatyczny powrót do BLE provisioningu; broker nieosiągalny (zgaszony lub zmienił IP) → po serii nieudanych prób węzeł ponawia wykrywanie brokera przez mDNS (`mqtt_reconnect_task`). Logika sprzętowa (alarm, klawiatura, RFID) działa lokalnie nawet bez MQTT.
- Logika sprzętowa działa w zadaniach FreeRTOS przypiętych do rdzenia 1; callbacki MQTT **nie mogą blokować** (np. moduł access deleguje cykl otwarcia zamka do osobnego wątku przez `xTaskNotifyGive`).

## Integracja z backendem / RPi (na przyszłość)

- **Auto-discovery:** backend subskrybuje `home/+/info` (retained) → otrzymuje listę obecnych węzłów z ich możliwościami od razu po starcie, bez konfiguracji.
- **Sterowanie:** publikacja na tematy `.../cmd` i moduł-specyficzne `.../set`.
- **Monitoring:** `home/+/availability` (online/offline) + `home/+/diag` (zdrowie).
- **Deployment plug-and-play na targach:** rekomendacja – RPi Zero 2W stawia własny AP (stały SSID) + broker; ESP-ki raz sprovisionowane do tego SSID łączą się automatycznie wszędzie, niezależnie od dostępu do internetu. mDNS (`rpi-smarthome`) zapewnia odnalezienie brokera bez znajomości IP. Skrypty RPi: katalog `scripts/`.

## Konwencje pracy

- Komentarze, logi i dokumentacja po polsku (bez polskich znaków w stringach logów ESP, by uniknąć problemów z kodowaniem na monitorze szeregowym).
- Nie używać `ESP_ERROR_CHECK` na operacjach, które mogą zawieść w czasie pracy (odczyt ADC, OTA) — błąd przerwałby pracę całego węzła. Obsługiwać błąd lokalnie i logować.
- Praca na branchu `develop`; główny branch to `main`.

## Znane ograniczenia / TODO (nie traktować jako gotowe)

- **Strona serwerowa (RPi) – zrobiona:** `scripts/` to kompleksowy plug-and-play pod Ubuntu Server (one-liner `bootstrap.sh`→`setup.sh`/`setup.d/`, idempotentny). Automatycznie: hostname `rpi-smarthome`, `avahi-daemon` (mDNS), konto brokera `smarthome`/`smarthome`, broker w Dockerze, usługi diagnostyczne + dashboard, Tailscale, watchdogi. Konfiguracja runtime: `/etc/smarthome/smarthome.env`; zarządzanie: CLI `smarthome`. **Uwaga:** instalator celowo NIE instaluje NetworkManagera (na Ubuntu Server siecią rządzi networkd; instalacja NM po Wi-Fi zrywała `wlan0`). AP fallback jest opt-in i wymaga ręcznej migracji Wi-Fi do NM. Zmieniając kontrakt serwera (tematy `home/system/server/*`, porty, konto brokera) aktualizuj [firmware/MQTT_API.md](firmware/MQTT_API.md).
- **Czas systemowy:** pole `"ts"` w telemetrii modułu environment używa `time()` bez synchronizacji SNTP → wartości są błędne dopóki nie dodano klienta NTP. Backend powinien stemplować czas po stronie serwera albo trzeba dodać SNTP w rdzeniu sieciowym.
- **Bezpieczeństwo:** dane brokera (`smarthome`/`smarthome`) i PoP są zaszyte w kodzie — akceptowalne dla makiety, nie dla produkcji.
- `backend/` i `frontend/` są puste — brak warstwy aplikacyjnej/UI.
- `firmware/src/CMakeLists.txt` opiera się na tym, że komponent `main` domyślnie widzi wszystkie komponenty; przy refaktorze warto dodać jawne `REQUIRES`.
