# Instrukcja Pierwszego Uruchomienia i Konfiguracji Wi-Fi (BLE Provisioning)

> 📦 Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [uruchomienie całości](../GETTING_STARTED.md) · [kontrakt MQTT](MQTT_API.md) · [serwer RPi](../scripts/README.md) · [OTA](OTA_UPDATE.md).

Niniejszy dokument opisuje procedurę konfiguracji połączenia sieciowego Wi-Fi w modułach ESP32 zainstalowanych na makiecie. Proces ten wykorzystuje technologię **Bluetooth Low Energy (BLE)** i eliminuje potrzebę wpisywania haseł sieciowych bezpośrednio do kodu źródłowego programu.

Dzięki wbudowanemu **mechanizmowi awaryjnego fallbacku**, jeśli makieta straci sygnał routera lub zostanie przeniesiona, mikrokontrolery po 5 nieudanych próbach samoczynnie podniosą serwery Bluetooth, umożliwiając zmianę sieci bez rozkręcania obudowy.

---

## 1. Wymagania Wstępne

Przed przystąpieniem do konfiguracji upewnij się, że posiadasz:
1. Smartfon z systemem **Android** lub **iOS** z włączoną obsługą **Bluetooth** oraz **Lokalizacją (GPS)**.
2. Zainstalowaną oficjalną aplikację mobilną **ESP BLE Provisioning** od firmy Espressif:
   * [Pobierz z Google Play (Android)](https://play.google.com/store/apps/details?id=com.espressif.provble)
   * [Pobierz z App Store (iOS)](https://apps.apple.com/us/app/esp-ble-provisioning/id1473535750)
3. Dane dostępowe do nowej sieci Wi-Fi (SSID oraz hasło). 
   * **UWAGA:** Układy ESP32 obsługują wyłącznie pasmo **2.4 GHz**.

---

## 2. Identyfikacja Modułów w Eterze (Nazwy Bluetooth)

Makieta składa się z trzech odrębnych układów ESP32 realizujących różne zadania. Aby skonfigurować Wi-Fi dla konkretnej sekcji makiety, musisz połączyć się z jej dedykowaną nazwą Bluetooth:

| Środowisko kompilacji w PlatformIO | Funkcja fizyczna na makiecie | Nazwa urządzenia w aplikacji BLE |
| :--- | :--- | :--- |
| `env:security_system` | System alarmowy, czujniki PIR, gazu, płomienia, syrena | **`PROV_Security`** |
| `env:access_system` | Kontrola dostępu, czytnik RFID, klawiatura, serwo | **`PROV_Access`** |
| `env:environment_system` | Stacja pogodowa, czujniki klimatu, wentylatory | **`PROV_Environment`** |
| `env:basic_default` | Czysty profil bazowy / testowy | **`PROV_default`** |

---

## 3. Procedura Konfiguracji Krok po Kroku

Tryb parowania (nasłuchiwanie Bluetooth) uruchamia się automatycznie: przy pierwszym uruchomieniu, po ręcznym wyczyszczeniu flasha, lub **samoczynnie po 5 nieudanych próbach połączenia ze starą siecią**.

1. **Włącz zasilanie makiety** (lub wybranego modułu ESP32).
2. Uruchom na smartfonie aplikację **ESP BLE Provisioning** i włącz Bluetooth.
3. Kliknij przycisk **Provision Device** (lub ikonę `+` w prawym górnym rogu).
4. Po zakończeniu skanowania, odszukaj na liście nazwę modułu, który chcesz skonfigurować (zgodnie z tabelą z Sekcji 2, np. **`PROV_Security`**).
5. Kliknij nazwę urządzenia, aby nawiązać z nim połączenie parujące.
6. Aplikacja wyświetli monit o podanie kodu autoryzacyjnego PIN (**Proof of Possession — PoP**). Wpisz uniwersalny klucz bezpieczeństwa dla tej makiety:
   * PIN (PoP): **`smarthome`**
7. Po pomyślnej autoryzacji aplikacja wyświetli listę sieci Wi-Fi wykrytych przez moduł ESP32.
8. Wybierz z listy docelową sieć Wi-Fi 2.4 GHz, wpisz jej hasło i kliknij przycisk **Provision**.

---

## 4. Weryfikacja Połączenia

Aplikacja mobilna prześle poświadczenia do ESP32, a na ekranie telefonu pojawią się trzy zielone ikony sukcesu:
* [x] *Sending Wi-Fi credentials*
* [x] *Applying Wi-Fi connection*
* [x] *Checking provisioning status*

W tym momencie moduł ESP32 trwale zapisuje poświadczenia w bezpiecznej pamięci nieulotnej (NVS), wyłącza układ Bluetooth w celu zwolnienia pamięci RAM, automatycznie odnajduje Raspberry Pi przez mDNS i nawiązuje bezpieczne połączenie z Brokerem MQTT.

---

## 5. Ręczne wymuszenie czyszczenia pamięci (Opcjonalnie)

Jeśli chcesz natychmiastowo wyczyścić zapisane dane sieciowe Wi-Fi i zmusić płytkę do ponownego parowania bez czekania na automatyczny timeout:

1. Podłącz wybrany moduł ESP32 kablem USB do komputera deweloperskiego.
2. Otwórz terminal w środowisku PlatformIO i wykonaj polecenie czyszczenia:
   ```bash
   pio run --target erase
   ```
3. Po wyświetleniu komunikatu [SUCCESS] odłącz kabel i zrestartuj zasilanie. Układ natychmiast przejdzie do Kroku 3 i pojawi się w eterze.

---

## 6. Provisioning zdalny vs. fizyczny (deployment w terenie)

**Ważne ograniczenie:** samo wpisanie poświadczeń Wi-Fi do nowego modułu odbywa się przez **Bluetooth**,
więc wymaga **fizycznej obecności** (telefon w zasięgu BLE). Tej części nie da się wykonać przez internet
ani z GitHub Actions. Można jednak **zdalnie wyzwolić** tryb parowania i tak zaprojektować wdrożenie,
by ponowny provisioning w terenie w ogóle nie był potrzebny.

### 6.1. Zdalne wymuszenie ponownego parowania
Gdy ktoś jest przy makiecie z telefonem, ale nie chcesz rozkręcać obudowy ani podłączać USB, możesz zdalnie
(przez SSH/Tailscale na Raspberry Pi) wysłać komendę `reset_wifi` — moduł skasuje zapisaną sieć i wróci do
trybu BLE:

```bash
smarthome cmd security reset_wifi      # albo: access / environment / system
# (równoważnie: mosquitto_pub -t home/security/cmd -m reset_wifi)
```

Działa to tylko gdy moduł jest jeszcze online (ma połączenie z brokerem). Jeśli moduł już stracił sieć,
zadziała automatyczny fallback z [sekcji 3](#3-procedura-konfiguracji-krok-po-kroku) (5 nieudanych prób → BLE).

### 6.2. Strategia „zero-touch" na targi: stała sieć z Raspberry Pi
Aby makieta była **w pełni plug-and-play również bez internetu** i nie wymagała provisioningu po przewiezieniu:

1. Sprovisionuj **raz** wszystkie trzy moduły do sieci o **stałym SSID/haśle**, którą zapewnia samo
   Raspberry Pi (Pi jako punkt dostępowy makiety).
2. Od tej pory ESP-ki zawsze znajdują tę samą sieć i tego samego brokera (mDNS `rpi-smarthome`) — niezależnie
   od miejsca i dostępu do internetu. Re-provisioning w terenie staje się zbędny.

Konfiguracja awaryjnego/stałego AP po stronie Pi oraz zdalne zarządzanie modułami: [scripts/README.md](../scripts/README.md).

> Podsumowanie: **pierwszy** provisioning = fizycznie przez BLE (raz, przy montażu). **Eksploatacja zdalna**
> = `reset_wifi`, diagnostyka i OTA przez MQTT/Tailscale, bez dotykania sprzętu.