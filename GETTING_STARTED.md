# Uruchomienie makiety od zera

Przewodnik dla osoby, która **dostaje makietę i serwer w swoje ręce** i chce uruchomić całość. Prowadzi przez całość: serwer (Raspberry Pi) → moduły (ESP32) → weryfikacja → sterowanie.

> Dokument część dokumentacji projektu — wróć do [README głównego](README.md), aby zobaczyć mapę pozostałych przewodników.

---

## Co składa się na makietę

| Element | Ilość | Rola |
| :------ | :---- | :--- |
| Płytka ESP32 „SECURITY" | 1 | Alarm: czujniki ruchu, gazu, ognia, syrena |
| Płytka ESP32 „ACCESS" | 1 | Dostęp: RFID, klawiatura, zamek (serwo) |
| Płytka ESP32 „ENVIRONMENT" | 1 | Klimat i zasilanie: BME280, światło, watomierze, wentylatory |
| Raspberry Pi (Zero 2W) | 1 | Serwer: broker MQTT, łączy wszystko |
| Zasilacze / okablowanie | – | 5 V do płytek i modułów |
| Smartfon (Android/iOS) | 1 | Jednorazowa konfiguracja Wi-Fi modułów przez Bluetooth |

> Jeśli elektronika **nie jest jeszcze podłączona** do płytek, najpierw zobacz [firmware/HARDWARE.md](firmware/HARDWARE.md) (co do którego pinu).

---

## Schemat działania (co się z czym łączy)

```
[ESP32 ×3]  --Wi-Fi 2.4GHz-->  [router / Access Point]  <--  [Raspberry Pi: broker MQTT]
     ^                                                              ^
     |  (mDNS: ESP-ki same znajdują "rpi-smarthome")                |
     +--------------------------------------------------------------+
```

Wszystkie urządzenia muszą być w **tej samej sieci Wi-Fi 2.4 GHz**. Pi udostępnia brokera MQTT, a ESP-ki same go odnajdują po nazwie `rpi-smarthome`.

---

## 1. Serwer: Raspberry Pi

> 🚧 **Uwaga:** strona serwerowa (skrypty RPi, automatyczny setup) jest **w trakcie budowy od zera** — docelowo cały setup Pi ma być w pełni plug-and-play (jednym poleceniem: hostname, mDNS, konto brokera, autostart). Poniższe kroki to **tymczasowa procedura ręczna**, która działa już teraz; po przebudowie skryptów ta sekcja zostanie zastąpiona prostszą instrukcją.

Cel: uruchomić brokera MQTT dostępnego jako `rpi-smarthome.local:1883`.

### 1.1. Przygotuj system
- Wgraj na kartę SD system (Ubuntu Server / Raspberry Pi OS, 64-bit) i połącz się przez SSH.
- Podłącz Pi do tej samej sieci Wi-Fi, w której ma pracować makieta (pasmo **2.4 GHz**).

### 1.2. Nadaj Pi nazwę `rpi-smarthome` i włącz mDNS  ⚠️ KLUCZOWE
Firmware ESP32 szuka brokera po nazwie `rpi-smarthome` przez mDNS. Bez tego moduły **nie znajdą serwera**.

```bash
sudo hostnamectl set-hostname rpi-smarthome
sudo apt update && sudo apt install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
```
Sprawdź z innego urządzenia w sieci: `ping rpi-smarthome.local` powinno odpowiadać.

### 1.3. Pobierz projekt
```bash
git clone https://github.com/JGZimek/smart-home-sns-automatyk.git
cd smart-home-sns-automatyk
```

### 1.4. Utwórz konto brokera (login/hasło)  ⚠️ WYMAGANE
Broker odrzuca anonimowe połączenia. Firmware loguje się jako **`esp32` / `esp32`**, więc trzeba utworzyć takie konto:

```bash
# wygeneruj plik hasła oczekiwany przez mosquitto/config/mosquitto.conf
docker run --rm -v "$PWD/mosquitto/config:/cfg" eclipse-mosquitto:2 \
  mosquitto_passwd -b -c /cfg/passwd esp32 esp32
```
(Alternatywnie, jeśli masz lokalnie `mosquitto_passwd`: `mosquitto_passwd -b -c mosquitto/config/passwd esp32 esp32`.)

### 1.5. Uruchom brokera
Najprościej przez Docker:
```bash
docker compose up -d        # lub: sudo docker-compose up -d
```
Sprawdź, że działa:
```bash
docker ps                   # kontener "mqtt_broker" powinien być Up
```

> **Pełna automatyczna instalacja** (broker + wyświetlacz LCD + awaryjny Access Point Wi-Fi + autostart) jest opisana w [scripts/README.md](scripts/README.md) (`./install.sh`). Uwaga: instalator zakłada projekt w `/home/smarthome/...` i **nie** ustawia jeszcze hostname/avahi z punktu 1.2 — zrób to ręcznie.

---

## 2. Moduły ESP32: firmware

Jeśli płytki **mają już wgrane firmware** (zostały zaprogramowane wcześniej) — pomiń ten krok i przejdź do [kroku 3](#3-konfiguracja-wi-fi-w-modułach-ble).

Jeśli musisz wgrać firmware samodzielnie:
1. Zainstaluj **VS Code + PlatformIO**.
2. Otwórz folder `firmware/`.
3. Podłącz daną płytkę USB i wgraj profil odpowiadający jej roli:
   ```bash
   cd firmware
   pio run -e security_system    -t upload     # płytka alarmowa
   pio run -e access_system      -t upload     # płytka dostępu
   pio run -e environment_system -t upload     # płytka klimatu
   ```
Szczegóły i opis profili: [firmware/README.md](firmware/README.md).

---

## 3. Konfiguracja Wi-Fi w modułach (BLE)

Każdą płytkę trzeba **raz** połączyć z Twoją siecią Wi-Fi. Robi się to przez Bluetooth, bez wpisywania haseł do kodu.

1. Zainstaluj na telefonie aplikację **ESP BLE Provisioning** (Espressif).
2. Włącz zasilanie płytki. Przy pierwszym starcie (lub po komendzie `reset_wifi`) rozgłasza się ona przez Bluetooth jako:
   - `PROV_Security`, `PROV_Access`, `PROV_Environment` (zależnie od roli).
3. W aplikacji: **Provision Device** → wybierz urządzenie → PIN (PoP): **`smarthome`** → wskaż sieć **2.4 GHz** i podaj hasło.
4. Płytka zapamięta sieć (NVS), zrestartuje się i sama połączy z brokerem.

Powtórz dla każdej z trzech płytek. Pełna instrukcja z ekranami: [firmware/WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md).

---

## 4. Weryfikacja: czy makieta żyje?

Na Raspberry Pi (lub dowolnym komputerze w sieci) podejrzyj ruch MQTT:

```bash
mosquitto_sub -h rpi-smarthome.local -u esp32 -P esp32 -t 'home/#' -v
```

Powinieneś zobaczyć m.in.:
```
home/security/availability ONLINE
home/security/info {"id":"esp32-...","kind":"security",...}
home/access/availability ONLINE
home/environment/availability ONLINE
home/garden/environment/temperature {"value":23.5,...}
home/<rodzaj>/diag {"uptime_s":...,"rssi":...}
```

Jeśli widzisz `ONLINE` i `info` dla wszystkich trzech modułów — **makieta działa**.

---

## 5. Sterowanie (szybki test)

```bash
# Uzbrój alarm
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 -t home/security/arm/set -m ARM

# Włącz wentylator chłodzący
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 -t home/garden/fan/cooling/set -m ON

# Otwórz zamek (na 5 s)
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 -t home/access/door/set -m OPEN

# Poproś moduł o diagnostykę / zrestartuj go
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 -t home/environment/cmd -m diag
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 -t home/access/cmd -m reboot
```

Komplet tematów i payloadów: [firmware/MQTT_API.md](firmware/MQTT_API.md).

---

## Rozwiązywanie problemów

| Objaw | Najczęstsza przyczyna / rozwiązanie |
| :---- | :---------------------------------- |
| Moduły nie pojawiają się w `mosquitto_sub` | (1) Pi nie nazywa się `rpi-smarthome` lub brak `avahi-daemon` (krok 1.2). (2) Płytka nie połączyła się z Wi-Fi (krok 3). (3) ESP i Pi w różnych sieciach. |
| Broker odrzuca połączenia | Brak pliku `mosquitto/config/passwd` lub złe konto (krok 1.4). Sprawdź `docker logs mqtt_broker`. |
| Aplikacja BLE nie widzi płytki | Płytka już sprovisionowana. Wymuś reset: zdalnie `mosquitto_pub -t home/<rodzaj>/cmd -m reset_wifi`, albo lokalnie `pio run -e <profil> -t erase`. |
| Płytka nie łączy się z Wi-Fi | Sieć musi być **2.4 GHz** (ESP32 nie obsługuje 5 GHz). |
| Moduł `ONLINE`, ale brak odczytów czujnika | Sprawdź podłączenie i logi serial (`115200`) – firmware wypisuje, które czujniki wykrył. Patrz [firmware/HARDWARE.md](firmware/HARDWARE.md). |
| Płytka nie startuje po podłączeniu klawiatury | Strapping pin GPIO12 – nie trzymaj klawiszy przy włączaniu. Patrz [firmware/HARDWARE.md](firmware/HARDWARE.md). |

> **Strona serwerowa do przebudowy:** docelowo kroki 1.2 (hostname + mDNS) i 1.4 (konto brokera) mają być częścią w pełni automatycznego, plug-and-play setupu Raspberry Pi (budowanego od zera). Do tego czasu wykonuje się je ręcznie jak wyżej.

---

## Co dalej

- Pełny opis komunikacji (dla integracji backendu): [firmware/MQTT_API.md](firmware/MQTT_API.md).
- Zdalna aktualizacja oprogramowania: [firmware/OTA_UPDATE.md](firmware/OTA_UPDATE.md).
- Architektura i rozwój firmware: [firmware/README.md](firmware/README.md).
