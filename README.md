# Smart Home – makieta (SNS Automatyk)

Interaktywna **makieta inteligentnego domu** sterowana przez **trzy niezależne płytki ESP32**, spięte lokalnym brokerem MQTT na **Raspberry Pi**, z planowaną aplikacją WWW (backend + frontend) do wizualizacji i sterowania.

Projekt jest tak zaprojektowany, by był **plug-and-play**: po jednorazowej konfiguracji makietę można włączyć w dowolnym miejscu (również bez internetu, np. na targach) i działa samodzielnie.

---

## Architektura

```
   ESP32 #1: SECURITY        ESP32 #2: ACCESS          ESP32 #3: ENVIRONMENT
   PIR · gaz · ogień         RFID · klawiatura         BME280 · TSL2591
   · syrena                  · serwo (zamek)           · INA219 · wentylatory
         \                          |                          /
          \                         |   Wi-Fi 2.4 GHz / MQTT   /
           \                        |                         /
        ┌─────────────────────────────────────────────────────────┐
        │   Raspberry Pi Zero 2W   →   rpi-smarthome.local          │
        │   • Broker MQTT (Mosquitto)  port 1883  +  9001 (WS)      │
        │   • mDNS · monitor węzłów + dashboard · CLI `smarthome`   │
        │   • fallbacki (AP Wi-Fi, watchdog) · Tailscale (zdalnie)  │
        └─────────────────────────────────────────────────────────┘
                                   │  MQTT
                       ┌───────────────────────────┐
                       │   Backend + Frontend       │   (w budowie)
                       │   panel WWW / sterowanie    │
                       └───────────────────────────┘
```

- Każda płytka to **autonomiczny węzeł**: sama łączy się z Wi-Fi (BLE provisioning), odnajduje brokera przez mDNS, ogłasza się (`home/<rodzaj>/info`), raportuje stan zdrowia i przyjmuje komendy. Logika sprzętowa działa lokalnie nawet bez brokera.
- Wszystkie płytki współdzielą **jeden kod firmware**; rola zależy od wybranego profilu kompilacji.

---

## Mapa repozytorium

| Katalog | Zawartość |
| :------ | :-------- |
| [`firmware/`](firmware/) | Firmware ESP32 (ESP-IDF + PlatformIO) – **główna część projektu**. |
| [`scripts/`](scripts/) | Plug-and-play serwer Raspberry Pi: instalator jednym poleceniem, broker, mDNS, CLI `smarthome`, diagnostyka węzłów + dashboard, awaryjny Wi-Fi, Tailscale. |
| [`mosquitto/`](mosquitto/) | Konfiguracja brokera MQTT (uruchamiany przez `docker-compose.yml`). |
| `backend/` | Aplikacja serwerowa – *placeholder, w budowie*. |
| `frontend/` | Interfejs WWW – *placeholder, w budowie*. |

---

## Dokumentacja – od czego zacząć?

Wybierz według tego, **co chcesz zrobić**:

| Twoja rola / cel | Przeczytaj |
| :--------------- | :--------- |
|  **Dostałem sprzęt i chcę uruchomić całość** | **[GETTING_STARTED.md](GETTING_STARTED.md)** |
|  Podłączam czujniki/elektronikę do ESP32 | [firmware/HARDWARE.md](firmware/HARDWARE.md) |
|  Piszę backend/frontend (sterowanie makietą) | [firmware/MQTT_API.md](firmware/MQTT_API.md) |
|  Rozwijam / kompiluję firmware | [firmware/README.md](firmware/README.md) |
|  Konfiguruję Wi-Fi w module ESP32 | [firmware/WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md) |
|  Aktualizuję firmware zdalnie (OTA) | [firmware/OTA_UPDATE.md](firmware/OTA_UPDATE.md) |
|  **Uruchamiam / obsługuję serwer RPi** (od zera, dostęp, zmiana sieci) | **[DEPLOYMENT.md](DEPLOYMENT.md)** |
|  Serwer RPi – referencja techniczna (CLI, usługi) | [scripts/README.md](scripts/README.md) |

---

## Szybki start (skrót)

1. **Serwer (RPi):** uruchom brokera MQTT i nadaj Pi nazwę `rpi-smarthome` → [GETTING_STARTED.md](GETTING_STARTED.md#1-serwer-raspberry-pi).
2. **Moduły (ESP32):** wgraj firmware (jeśli nie wgrane) i skonfiguruj Wi-Fi przez aplikację BLE → [GETTING_STARTED.md](GETTING_STARTED.md#3-konfiguracja-wi-fi-w-modułach-ble).
3. **Sprawdź:** `smarthome status` / dashboard `http://rpi-smarthome.local:8080` (lub `mosquitto_sub -t 'home/#' -v`) powinny pokazać moduły zgłaszające się jako `ONLINE`.

Pełna instrukcja krok po kroku: **[GETTING_STARTED.md](GETTING_STARTED.md)**.

---

## Status projektu

| Element | Stan |
| :------ | :--- |
| Firmware ESP32 (3 moduły + wspólny rdzeń) | działa, kompiluje się czysto |
| Sieć: BLE provisioning, mDNS, MQTT, OTA, diagnostyka |
| Broker MQTT (Mosquitto) | działa |
| Serwer RPi: plug-and-play (one-liner, hostname/mDNS, konto brokera, autostart, fallbacki) |
| Serwer RPi: CLI `smarthome`, dashboard diagnostyki węzłów, Tailscale |
| Backend (serwer aplikacyjny) | placeholder |
| Frontend (panel WWW) | placeholder |

Aktualna gałąź robocza: `develop`. Główna: `main`.
