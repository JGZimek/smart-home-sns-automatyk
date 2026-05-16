# Instrukcja Zdalnej Aktualizacji Oprogramowania (HTTP/MQTT OTA)

Niniejszy dokument opisuje procedurę bezprzewodowego wgrywania nowego oprogramowania (Over-the-Air) do konkretnych modułów ESP32 zamontowanych na makiecie. Architektura makiety obejmuje **trzy niezależne układy ESP32** wykonujące różne zadania. System aktualizacji został zaprojektowany tak, aby umożliwić bezdotykową aktualizację wybranego modułu bez ryzyka nadpisania kodu na pozostałych urządzeniach.

---

## 1. Architektura Adresowania Aktualizacji

Rozróżnienie urządzeń w sieci realizowane jest automatycznie na poziomie kompilacji warunkowej oraz dedykowanych kanałów **MQTT**. Każda płytka, w zależności od wgranego profilu oprogramowania, nasłuchuje na **własnym, unikalnym temacie aktualizacji**.

Przed wysłaniem oprogramowania należy określić cel, wybierając odpowiedni temat MQTT:

| Nazwa środowiska w PlatformIO | Funkcja modułu na makiecie | Dedykowany temat MQTT dla OTA |
| :--- | :--- | :--- |
| `env:security_system` | System alarmowy, czujniki PIR, gazu, ognia, syrena | **`home/security/update`** |
| `env:access_system` | Kontrola dostępu, czytnik RFID, klawiatura, serwo | **`home/access/update`** |
| `env:environment_system` | Stacja pogodowa, czujniki klimatu, wentylatory | **`home/environment/update`** |
| `env:basic_default` | Czysty profil deweloperski / testowy | **`home/system/update`** |

---

## 2. Wymagania Wstępne

1. Komputer deweloperski z zainstalowanym środowiskiem **VS Code + PlatformIO** podłączony do tej samej sieci lokalnej co makieta.
2. Zainstalowane środowisko **Python 3** na komputerze deweloperskim.
3. Działający i uruchomiony broker MQTT (serwer Mosquitto) na Raspberry Pi.

---

## 3. Procedura Wykonania Aktualizacji Krok po Kroku

### Krok 1: Wybór modułu i kompilacja
1. Otwórz projekt firmware w VS Code.
2. Na dolnym niebieskim pasku PlatformIO kliknij ikonę wyboru środowiska i wybierz docelowy moduł, który ma zostać zaktualizowany, np. `env:security_system`.
3. Kliknij ikonę **Ptasznika (Build)** na dolnym pasku lub użyj skrótu `Ctrl+Alt+B`.
4. **Nie używaj** ikony strzałki (**Upload**) — aktualizacja nie odbywa się przez USB.
5. Po pomyślnym zakończeniu kompilacji (`[SUCCESS]`) plik `firmware.bin` znajdzie się w katalogu:
   `.pio/build/[NAZWA_WYBRANEGO_SRODOWISKA]/firmware.bin`

### Krok 2: Uruchomienie lokalnego serwera HTTP
1. Otwórz terminal systemowy (CMD / PowerShell / Bash).
2. Przejdź do folderu z wykompilowanym plikiem `.bin`. Przykład dla modułu bezpieczeństwa:

```bash
cd firmware/.pio/build/security_system/
```

3. Uruchom serwer HTTP Pythona na porcie 8000:

```bash
python -m http.server 8000
```

4. Terminal wyświetli komunikat:
   `Serving HTTP on 0.0.0.0 port 8000 ...`
5. Pozostaw to okno terminala otwarte przez cały czas trwania aktualizacji.

### Krok 3: Odczytanie adresu IP komputera
Urządzenie ESP32 musi otrzymać dokładny adres IP komputera, z którego ma pobrać plik.

- **Windows**: Otwórz nowy terminal, wpisz `ipconfig` i odczytaj wartość **IPv4 Address** dla aktywnej karty sieciowej.
- **Linux / macOS**: Wpisz `ip a` lub `ifconfig` i odczytaj lokalny adres IP.

Przykładowy adres:
`192.168.1.15`

### Krok 4: Wysłanie rozkazu aktualizacji do wybranego urządzenia
Aby zaktualizować konkretną płytkę, należy opublikować link do pliku `firmware.bin` na temat MQTT przypisany do danego modułu.

#### Sposób A: Konsola Raspberry Pi (SSH)
Zaloguj się na Raspberry Pi przez SSH i wykonaj polecenie `mosquitto_pub`.

Przykład dla modułu bezpieczeństwa:

```bash
mosquitto_pub -h localhost -t "home/security/update" -m "http://192.168.1.15:8000/firmware.bin" -u "esp32" -P "esp32"
```

#### Sposób B: MQTT Explorer
1. Uruchom program **MQTT Explorer**.
2. Połącz się z brokerem MQTT działającym na Raspberry Pi.
3. W panelu publikacji ustaw:
- **Topic**: `home/security/update` lub odpowiednio `home/access/update` / `home/environment/update`
- **Payload**: `http://[IP_TWOJEGO_KOMPUTERA]:8000/firmware.bin`
4. Kliknij przycisk **Publish**.

---

## 4. Zachowanie Makiety i Diagnostyka

Po opublikowaniu komunikatu tylko urządzenie subskrybujące wskazany temat MQTT rozpocznie procedurę OTA. Moduł docelowy przerwie bieżące zadania, pobierze nowy firmware z serwera HTTP, zweryfikuje plik i po sukcesie wykona automatyczny restart.

W terminalu uruchomionego serwera HTTP pojawią się wpisy podobne do:
```text
GET /firmware.bin HTTP/1.1 200
```

Pozostałe dwa moduły ESP32 zignorują wiadomość, ponieważ nasłuchują na innych tematach MQTT. Ich działanie nie zostanie zakłócone.

---

## 5. Ochrona przed Błędem Operatora

Jeśli przez pomyłkę zostanie wysłany niewłaściwy plik binarny na temat przypisany do innego modułu, urządzenie pobierze plik, ale przed instalacją uruchomi mechanizmy weryfikacyjne frameworka **ESP-IDF**.

W przypadku wykrycia niezgodności lub błędu plik zostanie odrzucony, a urządzenie pozostanie przy dotychczasowej, sprawnej wersji oprogramowania. Dzięki temu aktualizacja nie powinna doprowadzić do uszkodzenia modułu ani utraty działania całej makiety.