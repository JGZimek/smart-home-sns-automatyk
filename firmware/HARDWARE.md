# Instrukcja Podłączenia Sprzętu (Pinout makiety)

> 📦 Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [uruchomienie całości](../GETTING_STARTED.md) · [firmware](README.md) · [API MQTT](MQTT_API.md).

Dokument dla osoby montującej elektronikę makiety. Opisuje, **co i do którego pinu** podłączyć na każdej z trzech płytek ESP32. Wszystkie przypisania pinów pochodzą wprost z kodu firmware (`src/*_system.cpp`) — jeśli zmienisz piny w kodzie, zaktualizuj też ten plik.

> **Płytka:** ESP32 DevKit (moduł `esp32dev`, układ ESP32-WROOM, 38 pinów). Numery `GPIO` to numery widoczne na opisach pinów płytki (np. „D14" = GPIO14).

---

## ⚠️ Zasady ogólne (przeczytaj przed lutowaniem)

1. **Wspólna masa (GND).** Wszystkie czujniki, moduły i zasilacze muszą mieć połączone GND z GND ESP32. Bez tego odczyty są losowe.
2. **Napięcia logiki:** ESP32 pracuje na **3.3 V**. Piny GPIO **nie są tolerancyjne na 5 V** — nie podawaj 5 V bezpośrednio na wejście GPIO.
3. **Zasilanie modułów:**
   - Serwo, syrena, przekaźniki, czujniki PIR — zasilaj z **5 V** (pin `VIN`/`5V` lub zewnętrzny zasilacz). Nie zasilaj serwa ani syreny z pinu `3V3` ESP32 (za słaby).
   - Czytnik **RC522 (RFID)** i czujniki **I2C (BME280, TSL2591)** — zasilaj **3.3 V** (RC522 zasilony 5 V ulega uszkodzeniu!).
4. **Piny tylko-wejściowe:** GPIO **34–39** nie mają wyjścia ani rezystorów podciągających — używaj ich wyłącznie jako wejścia (w makiecie: czujnik gazu na GPIO34).
5. **Pin startowy (strapping) GPIO12** — patrz ostrzeżenie przy module ACCESS.
6. ESP32 łączy się tylko z Wi-Fi **2.4 GHz**.

---

## 1. Płytka SECURITY (system alarmowy)

Środowisko: `env:security_system`. Kod: [src/security_system.cpp](src/security_system.cpp).

| Urządzenie                 | Sygnał        | GPIO ESP32 | Zasilanie | Uwagi |
| :------------------------- | :------------ | :--------- | :-------- | :---- |
| Czujnik ruchu PIR #1       | OUT           | **GPIO14** | 5 V       | Wejście, wewn. pull-up. Stan wysoki = ruch. |
| Czujnik ruchu PIR #2       | OUT           | **GPIO27** | 5 V       | jw. |
| Czujnik płomienia          | DO (cyfrowy)  | **GPIO26** | 3.3–5 V   | Wejście, pull-up. **Stan NISKI = wykryto płomień.** |
| Czujnik gazu (MQ-x)        | AO (analog)   | **GPIO34** | 5 V       | Wejście ADC1_CH6. Próg alarmu w kodzie: `GAS_THRESHOLD = 2000`. **Patrz uwaga poniżej.** |
| Syrena / brzęczyk          | + (sterowanie)| **GPIO25** | 5 V       | Wyjście. Stan wysoki = syrena gra. Większą syrenę sterować przez tranzystor/przekaźnik. |

> **Uwaga – czujnik gazu MQ:** wyjście analogowe modułów MQ potrafi sięgać ~5 V, a ADC ESP32 mierzy max ~3.3 V. Zastosuj **dzielnik napięcia** (np. 2× rezystor) na linii AO → GPIO34, albo ogranicz napięcie zasilania grzałki, aby nie przekroczyć 3.3 V na wejściu.

---

## 2. Płytka ENVIRONMENT (stacja pogodowa + zasilanie + wentylacja)

Środowisko: `env:environment_system`. Kod: [src/environment_system.cpp](src/environment_system.cpp).

### Magistrala I2C (wspólna dla czujników)

| Sygnał I2C | GPIO ESP32 |
| :--------- | :--------- |
| **SDA**    | **GPIO21** |
| **SCL**    | **GPIO22** |

Wszystkie poniższe czujniki podłącz **równolegle** do tych dwóch linii (+ 3.3 V + GND). Zalecane zewnętrzne rezystory podciągające 4.7 kΩ z SDA i SCL do 3.3 V (kod włącza też wewnętrzne pull-upy).

| Czujnik na I2C            | Adres I2C | Zasilanie | Funkcja |
| :------------------------ | :-------- | :-------- | :------ |
| BME280 / BMP280           | `0x76`    | 3.3 V     | Temperatura, ciśnienie (BME280 dodatkowo wilgotność). |
| TSL2591                   | `0x29`    | 3.3 V     | Natężenie światła (lux). |
| INA219 #1 „Solar"         | `0x45`    | 3.3 V     | Pomiar napięcia/prądu panelu słonecznego. |
| INA219 #2 „Battery"       | `0x44`    | 3.3 V     | Pomiar napięcia/prądu akumulatora. |

> Adresy INA219 ustawia się zworkami A0/A1 na module. Domyślny adres modułu to zwykle `0x40` — tutaj wymagane są `0x44` i `0x45`, więc odpowiednio zazworkuj oba moduły (są to różne adresy, by nie kolidowały na jednej magistrali).

### Wyjścia (przekaźniki wentylatorów)

| Urządzenie                   | Sygnał | GPIO ESP32 | Zasilanie | Uwagi |
| :--------------------------- | :----- | :--------- | :-------- | :---- |
| Przekaźnik – wentylator chłodzący | IN | **GPIO18** | moduł 5 V | **Sterowanie aktywne stanem NISKIM** (0 = ON). Po starcie wyłączony. |
| Przekaźnik – wentylator wentylacji | IN | **GPIO19** | moduł 5 V | jw. |

---

## 3. Płytka ACCESS (kontrola dostępu)

Środowisko: `env:access_system`. Kod: [src/access_system.cpp](src/access_system.cpp).

### Zamek – serwomechanizm

| Urządzenie | Sygnał | GPIO ESP32 | Zasilanie | Uwagi |
| :--------- | :----- | :--------- | :-------- | :---- |
| Serwo (zamek) | PWM (sygnał) | **GPIO4** | 5 V (osobno!) | PWM 50 Hz. 0° = zaryglowane, 90° = otwarte (auto-zamknięcie po 5 s). Zasilaj serwo z 5 V, GND wspólne z ESP32. |

### Czytnik RFID RC522 (magistrala SPI) — zasilanie **3.3 V**

| Pin RC522 | Sygnał | GPIO ESP32 |
| :-------- | :----- | :--------- |
| SDA / SS  | CS     | **GPIO5**  |
| SCK       | zegar  | **GPIO18** |
| MOSI      | dane → | **GPIO23** |
| MISO      | dane ← | **GPIO19** |
| 3.3V      | zasilanie | 3V3     |
| GND       | masa   | GND        |
| RST       | reset  | (opcjonalny — biblioteka działa bez podłączenia) |

### Klawiatura matrycowa 4×4 (8 pinów, bez zasilania)

Wiersze (`ROW`, wyjścia) i kolumny (`COL`, wejścia z pull-up):

| Linia klawiatury | GPIO ESP32 |
| :--------------- | :--------- |
| ROW 1            | **GPIO32** |
| ROW 2            | **GPIO33** |
| ROW 3            | **GPIO25** |
| ROW 4            | **GPIO26** |
| COL 1            | **GPIO27** |
| COL 2            | **GPIO14** |
| COL 3            | **GPIO12** |
| COL 4            | **GPIO13** |

Układ klawiszy (wiersz × kolumna):
```
1 2 3 A
4 5 6 B
7 8 9 C
* 0 # D
```

> **⚠️ Ostrzeżenie – GPIO12 (pin strapping):** GPIO12 jest pinem startowym ESP32. Jeśli przy włączaniu zasilania jest w stanie WYSOKIM, płytka może się nie uruchomić (wybiera złe napięcie flash). Kolumna klawiatury ma pull-up, więc gdy w momencie startu klawisz w 3. kolumnie jest wciśnięty, start może się zawiesić. W razie problemów z bootem nie trzymaj klawiszy podczas włączania; docelowo można przenieść `COL 3` na inny wolny pin (zmiana w `COL_PINS` w kodzie).

---

## Podsumowanie zajętości pinów

| GPIO | SECURITY        | ENVIRONMENT     | ACCESS          |
| :--- | :-------------- | :-------------- | :-------------- |
| 4    | –               | –               | Serwo PWM       |
| 5    | –               | –               | RFID CS         |
| 12   | –               | –               | Klaw. COL3 ⚠️    |
| 13   | –               | –               | Klaw. COL4      |
| 14   | PIR #1          | –               | Klaw. COL2      |
| 18   | –               | Przekaźnik chłodz. | RFID SCK     |
| 19   | –               | Przekaźnik went. | RFID MISO      |
| 21   | –               | I2C SDA         | –               |
| 22   | –               | I2C SCL         | –               |
| 23   | –               | –               | RFID MOSI       |
| 25   | Syrena          | –               | Klaw. ROW3      |
| 26   | Czujnik płomienia | –             | Klaw. ROW4      |
| 27   | PIR #2          | –               | Klaw. COL1      |
| 32   | –               | –               | Klaw. ROW1      |
| 33   | –               | –               | Klaw. ROW2      |
| 34   | Czujnik gazu (ADC) | –            | –               |

Każda kolumna to **osobna płytka ESP32** — te same numery GPIO na różnych płytkach nie kolidują, bo to fizycznie różne układy.

---

## Po podłączeniu

1. Wgraj firmware dla danej roli: `pio run -e <środowisko> -t upload` (patrz [README.md](README.md)).
2. Skonfiguruj Wi-Fi przez aplikację BLE (patrz [WIFI_PROVISIONING.md](WIFI_PROVISIONING.md)).
3. Sprawdź logi na monitorze szeregowym (`115200`): każdy moduł wypisuje, które czujniki wykrył (np. „TSL2591 odnaleziony", „Nie odnaleziono BME280").
