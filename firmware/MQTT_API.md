# Kontrakt komunikacyjny MQTT (dla backendu)

> Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [uruchomienie całości](../GETTING_STARTED.md) · [firmware](README.md) · [pinout](HARDWARE.md).

Dokument dla osoby piszącej backend/frontend sterujący makietą. Opisuje **wszystkie** kanały MQTT, którymi można obserwować i sterować trzema płytkami ESP32. Źródłem prawdy jest kod firmware (`src/*.cpp`) — przy zmianach w firmware aktualizuj ten plik.

---

## 1. Połączenie z brokerem

| Parametr | Wartość |
| :------- | :------ |
| Broker   | Mosquitto na Raspberry Pi (`docker-compose.yml` w katalogu głównym) |
| Host     | `rpi-smarthome.local` (mDNS) lub IP RPi; port **1883** |
| Login / hasło | `smarthome` / `smarthome` |
| WebSocket | port **9001** (np. dla frontendu w przeglądarce) |

> Każda płytka sama odnajduje brokera przez mDNS (`rpi-smarthome`) i łączy się automatycznie — backend nie konfiguruje urządzeń, tylko korzysta z tych samych tematów.

---

## 2. Przestrzeń nazw i konwencje

- Baza każdego węzła: **`home/<rodzaj>`**, gdzie `<rodzaj>` ∈ `security`, `access`, `environment`, `system`. Pod `system` mieszczą się: profil testowy ESP32 (`home/system/info`…) oraz **serwer Raspberry Pi**, który zgłasza się jako węzeł pod `home/system/server/...` (patrz [§6a](#6a-węzeł-serwera-raspberry-pi)).
- **Kierunek** w tabelach jest z punktu widzenia ESP32: `pub` = ESP publikuje (backend subskrybuje), `sub` = ESP nasłuchuje (backend publikuje, żeby sterować).
- **Retained** (zachowane na brokerze) używane dla stanów, które backend powinien znać natychmiast po starcie (availability, info, stany aktuatorów, ostatni alarm). Zdarzenia chwilowe (ruch, naciśnięcie klawisza, telemetria) nie są retained.
- Payloady stanów/telemetrii to **JSON**; proste komendy/stany aktuatorów bywają zwykłym tekstem (`ON`, `OPEN`, `ARM`).

---

## 3. Interfejs wspólny (każda płytka, niezależnie od rodzaju)

Implementacja: `src/device_core.cpp`, `src/mqtt_core.cpp`.

| Temat | Kier. | Retained | Payload / przykład |
| :---- | :---- | :------- | :----------------- |
| `home/<rodzaj>/info` | pub | ✅ | `{"id":"esp32-a1b2c3","kind":"security","name":"SECURITY SYSTEM","fw":"2.1.0","idf":"v5.5.0","ip":"192.168.1.42","cmd_topic":"home/security/cmd","diag_topic":"home/security/diag"}` |
| `home/<rodzaj>/availability` | pub | ✅ | `ONLINE` / `OFFLINE` (`OFFLINE` ustawia broker jako Last Will przy zaniku węzła) |
| `home/<rodzaj>/diag` | pub | ❌ | co ~30 s: `{"id":"esp32-a1b2c3","uptime_s":3600,"heap":210000,"min_heap":180000,"rssi":-58,"reset":"poweron"}` |
| `home/<rodzaj>/cmd` | sub | – | komenda zarządcza (patrz niżej) |
| `home/<rodzaj>/update` | sub | – | OTA: URL do `firmware.bin` (patrz [OTA_UPDATE.md](OTA_UPDATE.md)) |

**Komendy (`home/<rodzaj>/cmd`)** — payload jako słowo kluczowe lub JSON `{"cmd":"..."}`:

| Komenda | Efekt |
| :------ | :---- |
| `reboot` | Restart węzła. |
| `reset_wifi` | Kasuje zapisaną sieć Wi-Fi i restartuje do BLE provisioningu (rekonfiguracja telefonem). |
| `set_wifi` | Zapisuje **nowe** dane Wi-Fi i restartuje na nową sieć — bez telefonu. JSON: `{"cmd":"set_wifi","ssid":"...","pass":"..."}`. |
| `identify` | Ponownie publikuje `info`. |
| `diag` / `ping` | Natychmiast publikuje `diag`. |

Pola `diag`: `uptime_s` (sekundy), `heap`/`min_heap` (bajty wolnej pamięci, bieżąco i minimum), `rssi` (dBm), `reset` (`poweron`/`sw`/`panic`/`brownout`/`task_wdt`/...).

**Przepięcie całej makiety na nową sieć** — z serwera RPi jednym poleceniem:
`sudo smarthome wifi switch-all "<SSID>" "<HASLO>"`. Rozsyła `set_wifi` do wszystkich węzłów
(muszą być online), a po chwili przełącza sam RPi. Kolejność ma znaczenie: węzły muszą być
osiągalne w obecnej sieci w chwili wysyłki. Kto był offline — wróci do BLE po zaniku starej sieci.

---

## 4. Moduł SECURITY (`home/security/...`)

System alarmowy. Kod: `src/security_system.cpp`.

### Sterowanie (backend → ESP)
| Temat | Payload | Opis |
| :---- | :------ | :--- |
| `home/security/arm/set` | `ARM` / `DISARM` | Uzbraja / rozbraja alarm. Rozbrojenie wyłącza syrenę. |

### Telemetria / zdarzenia (ESP → backend)
| Temat | Retained | Payload | Kiedy |
| :---- | :------- | :------ | :---- |
| `home/security/status` | ✅ | `{"val":1}` uzbrojony / `{"val":0}` rozbrojony | przy zmianie stanu uzbrojenia |
| `home/security/motion/1` | ❌ | `{"val":1}` | wykrycie ruchu PIR #1 (tylko gdy uzbrojony) |
| `home/security/motion/2` | ❌ | `{"val":1}` | wykrycie ruchu PIR #2 (tylko gdy uzbrojony) |
| `home/security/fire` | ✅ | `{"val":1}` / `{"val":0}` | zmiana stanu czujnika płomienia (zawsze aktywny) |
| `home/security/gas` | ✅ | `{"val":<0–4095>}` | zmiana progu gazu (zawsze aktywny); wartość = surowy odczyt ADC |

> Czujniki ognia i gazu działają niezależnie od uzbrojenia. Syrena włącza się lokalnie przy każdym zdarzeniu — backend nie musi nią sterować (choć rozbrojenie ją wycisza).

---

## 5. Moduł ENVIRONMENT (`home/garden/...`)

Stacja pogodowa, pomiar zasilania, wentylacja. Kod: `src/environment_system.cpp`. Telemetria publikowana cyklicznie **co 5 s**.

> Uwaga: ten moduł używa bazy tematów `home/garden/...` dla danych domenowych, natomiast jego tematy systemowe (info/diag/cmd/availability/update) są pod `home/environment/...`.

### Sterowanie (backend → ESP)
| Temat | Payload | Opis |
| :---- | :------ | :--- |
| `home/garden/fan/cooling/set` | `ON` / `OFF` | Wentylator chłodzący. |
| `home/garden/fan/vent/set` | `ON` / `OFF` | Wentylator wentylacji. |

### Telemetria (ESP → backend)
| Temat | Retained | Payload | Jednostka |
| :---- | :------- | :------ | :-------- |
| `home/garden/fan/cooling/state` | ✅ | `ON` / `OFF` | stan przekaźnika |
| `home/garden/fan/vent/state` | ✅ | `ON` / `OFF` | stan przekaźnika |
| `home/garden/environment/temperature` | ❌ | `{"value":23.5,"ts":...}` | °C |
| `home/garden/environment/humidity` | ❌ | `{"value":48.2,"ts":...}` | % (tylko gdy BME280, nie BMP280) |
| `home/garden/environment/pressure` | ❌ | `{"value":1013.2,"ts":...}` | hPa |
| `home/garden/environment/light` | ❌ | `{"value":350.0,"ts":...}` | lux |
| `home/garden/power/solar/voltage` | ❌ | `{"value":5.02,"ts":...}` | V |
| `home/garden/power/solar/current` | ❌ | `{"value":0.1234,"ts":...}` | A |
| `home/garden/power/solar/power` | ❌ | `{"value":0.6200,"ts":...}` | W |
| `home/garden/power/battery/voltage` | ❌ | `{"value":3.90,"ts":...}` | V |
| `home/garden/power/battery/current` | ❌ | `{"value":0.0500,"ts":...}` | A |
| `home/garden/power/battery/power` | ❌ | `{"value":0.1950,"ts":...}` | W |

> **`ts`** to znacznik czasu epoch z zegara ESP32. Obecnie **niewiarygodny** (brak synchronizacji NTP w firmware) — backend powinien stemplować czas własnym zegarem przy odbiorze.

---

## 6. Moduł ACCESS (`home/access/...`)

Kontrola dostępu. Kod: `src/access_system.cpp`.

### Sterowanie (backend → ESP)
| Temat | Payload | Opis |
| :---- | :------ | :--- |
| `home/access/door/set` | `OPEN` | Otwiera zamek na 5 s, po czym automatycznie zamyka. |

### Zdarzenia / stany (ESP → backend)
| Temat | Retained | Payload | Kiedy |
| :---- | :------- | :------ | :---- |
| `home/access/door/state` | ✅ | `OPEN` / `CLOSED` | przy otwarciu i zamknięciu zamka |
| `home/access/rfid` | ❌ | `{"uid":"123456789"}` | zbliżenie karty/tagu RFID |
| `home/access/keypad` | ❌ | `{"key":"5"}` | naciśnięcie klawisza (znaki `0-9 A-D * #`) |

> Weryfikacja kodów PIN i autoryzacja kart to **logika backendu** — firmware tylko zgłasza surowe zdarzenia (`rfid`, `keypad`) i wykonuje rozkaz `door/set OPEN`. Backend decyduje, czy otworzyć.

---

## 6a. Węzeł serwera (Raspberry Pi)

To **nie firmware**, lecz serwer RPi — ale w MQTT zachowuje się jak każdy inny węzeł, więc backend wykrywa go tą samą drogą (`home/+/info`). Publikuje go usługa `smarthome-node-monitor` (Bash/Python na Pi, patrz [scripts/README.md](../scripts/README.md)). Dzięki temu zdrowie serwera widać tak samo jak zdrowie ESP-ek.

| Temat | Kier. | Retained | Payload / przykład |
| :---- | :---- | :------- | :----------------- |
| `home/system/server/info` | pub | ✅ | `{"id":"rpi-rpi-smarthome","kind":"system","name":"Smart Home Server (RPi)","role":"broker+monitor","host":"rpi-smarthome"}` |
| `home/system/server/availability` | pub | ✅ | `ONLINE` / `OFFLINE` (`OFFLINE` jako Last Will, gdy monitor padnie) |
| `home/system/server/diag` | pub | ❌ | co ~30 s: `{"uptime_s":...,"host_uptime_s":...,"cpu_temp_c":47.1,"mem_total_mb":480,"mem_avail_mb":260,"ip":"192.168.1.10","tailscale_ip":"100.x.y.z","ts":"2026-06-11T10:00:00+00:00"}` |

> Serwer **agreguje** też stan wszystkich węzłów do `/var/lib/smarthome/nodes.json` i wystawia go jako dashboard HTTP (`http://rpi-smarthome.local:8080`, JSON pod `/api/nodes`). To wygodna alternatywa dla samodzielnej subskrypcji `home/+/diag` po stronie backendu.

---

## 7. Typowy scenariusz integracji

1. **Połącz** się z brokerem (`rpi-smarthome.local:1883`, `smarthome`/`smarthome`).
2. **Wykryj urządzenia:** zasubskrybuj `home/+/info` i `home/+/availability` (retained → dostaniesz aktualny stan od razu). Z `info` masz id, rodzaj, wersję fw, IP oraz tematy `cmd`/`diag`.
3. **Obserwuj** telemetrię/zdarzenia z tabel modułów oraz `home/+/diag` (zdrowie).
4. **Steruj** publikując na tematy `.../set` i `.../cmd`.

### Przykłady (mosquitto_clients)
```bash
# Podgląd wszystkiego
mosquitto_sub -h rpi-smarthome.local -u smarthome -P smarthome -t 'home/#' -v

# Auto-discovery: kto jest online i czym jest
mosquitto_sub -h rpi-smarthome.local -u smarthome -P smarthome -t 'home/+/info' -t 'home/+/availability' -v

# Uzbrojenie alarmu
mosquitto_pub -h rpi-smarthome.local -u smarthome -P smarthome -t home/security/arm/set -m ARM

# Włączenie wentylatora chłodzącego
mosquitto_pub -h rpi-smarthome.local -u smarthome -P smarthome -t home/garden/fan/cooling/set -m ON

# Otwarcie drzwi
mosquitto_pub -h rpi-smarthome.local -u smarthome -P smarthome -t home/access/door/set -m OPEN

# Diagnostyka na żądanie / restart węzła
mosquitto_pub -h rpi-smarthome.local -u smarthome -P smarthome -t home/environment/cmd -m diag
mosquitto_pub -h rpi-smarthome.local -u smarthome -P smarthome -t home/security/cmd -m reboot
```

> **Z poziomu samego Pi** te same operacje upraszcza CLI `smarthome` (cienka nakładka na powyższe tematy): `smarthome nodes`, `smarthome cmd security reboot`, `smarthome ota environment <url>`, `smarthome set home/access/door/set OPEN`. Szczegóły: [scripts/README.md](../scripts/README.md).

---

## 8. Pełna lista tematów (ściąga)

```
# WSPÓLNE (rodzaj = security | access | environment | system)
home/<rodzaj>/info            pub  retained   JSON tożsamości
home/<rodzaj>/availability    pub  retained   ONLINE | OFFLINE
home/<rodzaj>/diag            pub             JSON zdrowia (~30 s)
home/<rodzaj>/cmd             sub             reboot | reset_wifi | set_wifi{ssid,pass} | identify | diag
home/<rodzaj>/update          sub             URL firmware (OTA)

# SECURITY
home/security/arm/set         sub             ARM | DISARM
home/security/status          pub  retained   {"val":0|1}
home/security/motion/1        pub             {"val":1}
home/security/motion/2        pub             {"val":1}
home/security/fire            pub  retained   {"val":0|1}
home/security/gas             pub  retained   {"val":<adc>}

# ENVIRONMENT  (dane pod home/garden/*, system pod home/environment/*)
home/garden/fan/cooling/set       sub         ON | OFF
home/garden/fan/vent/set          sub         ON | OFF
home/garden/fan/cooling/state     pub retained ON | OFF
home/garden/fan/vent/state        pub retained ON | OFF
home/garden/environment/temperature  pub      {"value":..,"ts":..}  °C
home/garden/environment/humidity     pub      {"value":..,"ts":..}  %
home/garden/environment/pressure     pub      {"value":..,"ts":..}  hPa
home/garden/environment/light        pub      {"value":..,"ts":..}  lux
home/garden/power/solar/voltage      pub      {"value":..,"ts":..}  V
home/garden/power/solar/current      pub      {"value":..,"ts":..}  A
home/garden/power/solar/power        pub      {"value":..,"ts":..}  W
home/garden/power/battery/voltage    pub      {"value":..,"ts":..}  V
home/garden/power/battery/current    pub      {"value":..,"ts":..}  A
home/garden/power/battery/power      pub      {"value":..,"ts":..}  W

# ACCESS
home/access/door/set          sub             OPEN
home/access/door/state        pub  retained   OPEN | CLOSED
home/access/rfid              pub             {"uid":"<serial>"}
home/access/keypad            pub             {"key":"<znak>"}

# SERWER (Raspberry Pi – publikowane przez smarthome-node-monitor)
home/system/server/info           pub retained JSON tożsamości serwera
home/system/server/availability   pub retained ONLINE | OFFLINE
home/system/server/diag           pub          JSON zdrowia Pi (~30 s)
```
