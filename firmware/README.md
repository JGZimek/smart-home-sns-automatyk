# Smart Home - Oprogramowanie Mikrokontrolerow (Firmware)

Ten katalog zawiera kody zrodlowe (firmware) dla urzadzen koncowych systemu Smart Home. Wszystkie projekty zostaly napisane w jezyku C++ z wykorzystaniem frameworka Arduino i srodowiska PlatformIO. Kody sa przystosowane do dzialania na mikrokontrolerach ESP32.

Kazdy modul wykorzystuje system czasu rzeczywistego (FreeRTOS) do rozdzielenia zadan obslugi czujnikow (rdzen 1) oraz komunikacji sieciowej MQTT i WiFi (rdzen 0), co zapewnia stabilnosc i brak opoznien.

## Struktura projektow

Katalog podzielony jest na trzy niezalezne projekty PlatformIO:

### 1. access-system (System Kontroli Dostepu)
Odpowiada za weryfikacje uzytkownikow i sterowanie zamkiem do drzwi.
- Sprzet: Czytnik RFID (MFRC522), klawiatura matrycowa 4x4, serwomechanizm (zamek).
- Dzialanie: Wysyla odczytane karty i kody PIN do brokera MQTT. Pozwala na zdalne i lokalne otwieranie drzwi.
- Tryb awaryjny sieci (WiFiManager): `SmartHome-Access`

### 2. environment-system (Stacja Pogodowa i Zasilanie)
Odpowiada za monitorowanie warunkow srodowiskowych, parametrow zasilania oraz sterowanie wentylatorami.
- Sprzet: Czujnik BME280 (temperatura, wilgotnosc, cisnienie), czujnik swiatla TSL2591, 2x watomierz INA219 (monitorowanie panelu slonecznego i akumulatora), przekazniki do wentylatorow (chlodzenie i wentylacja).
- Dzialanie: Cyklicznie publikuje parametry srodowiskowe i elektryczne. Odbiera komendy MQTT do zalaczania wentylatorow.
- Tryb awaryjny sieci (WiFiManager): `SmartHome-Env`

### 3. security-system (System Alarmowy)
Odpowiada za wykrywanie zagrozen (wlamanie, pozar, wyciek gazu) i wyzwalanie syreny.
- Sprzet: 2x czujnik ruchu PIR, czujnik plomienia, czujnik gazu (MQ-x), syrena alarmowa.
- Dzialanie: Nasluchuje komend uzbrajania/rozbrajania systemu. Stale monitoruje czujniki dymu i gazu (niezaleznie od uzbrojenia), a stan czujnikow PIR bierze pod uwage tylko w trybie uzbrojonym.
- Tryb awaryjny sieci (WiFiManager): `SmartHome-Security`

## Konfiguracja i wgrywanie

Aby skompilowac i wgrac oprogramowanie na ESP32:

1. Zainstaluj srodowisko Visual Studio Code wraz z rozszerzeniem PlatformIO.
2. Otworz jeden z trzech folderow (np. `firmware/access-system`) w VS Code (nie otwieraj glownego folderu `firmware`, kazdy podfolder to osobny projekt).
3. Podlacz ESP32 do komputera przewodem USB.
4. Kliknij ikone strzalki "Upload" na dolnym pasku PlatformIO. Wszystkie potrzebne biblioteki zostana pobrane automatycznie zgodnie z plikiem `platformio.ini`.

## Pierwsze uruchomienie w docelowej sieci (WiFiManager)

Urzadzenia nie maja wpisanych na sztywno hasel do WiFi. 

1. Po pierwszym uruchomieniu mikrokontrolera stworzy on wlasna, otwarta siec WiFi (np. `SmartHome-Access`, `SmartHome-Env`).
2. Polacz sie z nia z telefonu lub komputera.
3. Powinien automatycznie otworzyc sie panel konfiguracyjny (tzw. Captive Portal). Jesli nie, wejdz na adres `http://192.168.4.1`.
4. Podaj nazwe (SSID) i haslo do swojej domowej sieci WiFi.
5. W specjalnym polu podaj adres IP Twojego brokera MQTT (czyli adres lokalny Twojego Raspberry Pi Zero).
6. Zapisz ustawienia. Urzadzenie zrestartuje sie, polaczy z Twoja siecia i zacznie komunikowac sie z systemem glownym.
