# Smart Home RPi - Skrypty Instalacyjne i Konfiguracyjne

> 📦 Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [uruchomienie całości](../GETTING_STARTED.md).
>
> 🚧 **Uwaga:** ta strona serwerowa (skrypty RPi) jest **w trakcie przebudowy od zera** na kompleksowy plug-and-play. Obecna zawartość to wersja tymczasowa; opisane niżej kroki działają, ale docelowa architektura setupu RPi dopiero powstaje.

Ten folder zawiera zautomatyzowane narzedzia (Infrastructure as Code) do postawienia i konfiguracji glownego serwera Smart Home (brokera MQTT, uslug pomocniczych) na Raspberry Pi. Skrypty sa zoptymalizowane pod katem dzialania na Raspberry Pi Zero 2 W z systemem Ubuntu Server.

## Szybki start (Instalacja na czystym systemie)

Jesli masz swiezo wgrany system Ubuntu Server na karcie SD i jestes polaczony przez SSH, wykonaj ponizsze kroki:

1. Pobierz repozytorium i przejdz do odpowiedniej galezi:
   ```bash
   git clone [https://github.com/JGZimek/smart-home-sns-automatyk.git](https://github.com/JGZimek/smart-home-sns-automatyk.git)
   cd smart-home-sns-automatyk
   git checkout develop
   ```

2. Przejdz do folderu skryptow i nadaj uprawnienia do wykonywania:
   ```bash
   cd scripts
   chmod +x install.sh diag.sh host_config.sh
   ```

3. Uruchom instalator (One-Click):
   ```bash
   ./install.sh
   ```

4. Sprawdz poprawnosc instalacji:
   ```bash
   ./diag.sh
   ```
   *Wszystkie uslugi powinny miec status "OK" lub "Dziala".*

5. (Opcjonalnie) Uruchom ponownie system, aby upewnic sie, ze wszystko wstaje z autostartu:
   ```bash
   sudo reboot
   ```

## Struktura katalogu

- `install.sh` - Glowny instalator. Aktualizuje system, instaluje Dockera i narzedzia, uruchamia optymalizacje, buduje srodowisko wirtualne dla Pythona i aktywuje uslugi systemd.
- `host_config.sh` - Skrypt optymalizujacy system operacyjny (zwieksza SWAP do 1024MB, wylacza oszczedzanie energii dla WiFi zapobiegajac rozlaczeniom, konfiguruje sprzetowego Watchdoga).
- `diag.sh` - Szybkie narzedzie do monitorowania. Wyswietla status Dockera, serwisow systemowych, temperature CPU, zajetosc SWAP oraz aktualne adresy IP.
- `src/` - Kody zrodlowe w jezyku Python:
  - `broker_lcd.py` - Obsluga fizycznego wyswietlacza LCD (I2C), informujaca o IP i stanie brokera.
  - `wifi_manager.py` - Menedzer awaryjny sieci. W przypadku braku internetu tworzy wlasny Access Point (`SmartHome-Config`), aby umozliwic konfiguracje nowej sieci przez przegladarke.
- `services/` - Pliki konfiguracyjne uslug `systemd` (`broker_lcd.service`, `wifi_manager.service`), ktore sa kopiowane do systemu podczas instalacji, aby zapewnic ciagle dzialanie w tle i autostart.

## Diagnozowanie problemow

Jesli usluga `diag.sh` wskaze na blad z ktoryms z komponentow, mozesz przejrzec jego logi za pomoca polecenia `journalctl`, np.:

* Logi wyswietlacza LCD:
  ```bash
  journalctl -u broker_lcd -f
  ```
* Logi menedzera WiFi:
  ```bash
  journalctl -u wifi_manager -f
  ```
