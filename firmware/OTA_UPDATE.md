# Zdalna aktualizacja firmware (OTA przez HTTP/MQTT)

> 📦 Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [kontrakt MQTT](MQTT_API.md) · [serwer RPi](../scripts/README.md) · [firmware](README.md).

Bezprzewodowa aktualizacja wybranego modułu ESP32 **bez dotykania makiety**. Każda płytka nasłuchuje
na własnym temacie MQTT i pobiera nowy `firmware.bin` po **HTTP** z dowolnego hosta widocznego w jej sieci.
Pozostałe moduły ignorują komunikat (inny temat), więc aktualizacja jednego węzła nie rusza reszty.

---

## 1. Jak to działa (mechanizm)

```
[builder: PlatformIO]  --firmware.bin-->  [host HTTP]  <==HTTP pobieranie==  [ESP32 docelowy]
                                              ^                                    ^
        publikacja URL na MQTT  home/<kind>/update  ------------------------------+
```

1. Budujesz `firmware.bin` dla profilu odpowiadającego roli płytki.
2. Plik trafia na **host HTTP osiągalny z sieci makiety** (zalecane: sam Raspberry Pi – patrz §3).
3. Publikujesz URL pliku na temat `home/<kind>/update`.
4. Tylko węzeł nasłuchujący ten temat pobiera obraz w osobnym wątku (`ota_update_task`,
   `src/network_core.cpp`), weryfikuje go mechanizmami ESP-IDF i restartuje się na nową wersję.

> **HTTP, nie HTTPS.** Klient OTA w firmware (`esp_http_client`) pobiera obraz po zwykłym HTTP, więc URL
> musi być `http://...`. To akceptowalne w lokalnej sieci makiety; nie wystawiaj hosta firmware do internetu.

### Adresowanie modułów

| Profil PlatformIO | Rola | Temat OTA |
| :--- | :--- | :--- |
| `env:security_system` | Alarm (PIR, gaz, ogień, syrena) | **`home/security/update`** |
| `env:access_system` | Dostęp (RFID, klawiatura, serwo) | **`home/access/update`** |
| `env:environment_system` | Klimat / zasilanie / wentylacja | **`home/environment/update`** |
| `env:basic_default` | Profil testowy | **`home/system/update`** |

---

## 2. Budowanie obrazu

```bash
cd firmware
pio run -e security_system          # -> .pio/build/security_system/firmware.bin
```

Plik wynikowy: `firmware/.pio/build/<środowisko>/firmware.bin`. Nie używaj `-t upload` (to wgrywanie przez USB).

---

## 3. Zalecany sposób: hosting na Raspberry Pi + CLI `smarthome` (zdalnie)

Raspberry Pi jest zawsze w tej samej sieci co ESP-ki i serwuje pliki firmware po HTTP
(usługa `smarthome-node-monitor`, katalog `/var/lib/smarthome/firmware`, port `WEB_PORT`, domyślnie 8080).
Dzięki temu **cała operacja jest zdalna** – łączysz się z Pi przez SSH/Tailscale i wydajesz jedno polecenie.

```bash
# 1. Skopiuj zbudowany obraz na Pi (przez Tailscale lub LAN):
scp firmware/.pio/build/security_system/firmware.bin  rpi-smarthome:/tmp/security.bin

# 2. Na Pi: rozgłoś OTA do modułu (CLI hostuje plik i publikuje właściwy URL):
ssh rpi-smarthome 'smarthome ota security /tmp/security.bin'
```

`smarthome ota <kind> <plik>`:
- kopiuje `<plik>` do `/var/lib/smarthome/firmware/`,
- buduje URL `http://<IP-Pi>:8080/firmware/<nazwa>` (IP, nie `.local` – pewne dla klienta HTTP w ESP),
- publikuje go na `home/<kind>/update`.

Można też podać gotowy URL (np. inny serwer w LAN): `smarthome ota access http://192.168.1.50:8000/firmware.bin`.

> Wgrywanie pliku do katalogu firmware wymaga prawa zapisu – działa dla użytkownika w grupie `smarthome`
> albo przez `sudo smarthome ota ...`.

Śledzenie postępu:
```bash
smarthome watch 'home/security/#'     # ruch MQTT modułu
smarthome logs broker                 # log brokera
# oraz log szeregowy płytki (115200): linie z tagiem "OTA"
```

---

## 4. W pełni zdalnie z GitHub Actions (przez Tailscale)

Sieć makiety bywa odcięta od internetu, ale Raspberry Pi jest osiągalne przez **Tailscale**. Workflow
[`.github/workflows/firmware-ota.yml`](../.github/workflows/firmware-ota.yml) (uruchamiany ręcznie –
`workflow_dispatch`) realizuje: build → wejście do tailnetu → `scp` obrazu na Pi → `smarthome ota`.

Schemat:
```
GitHub Actions:  pio run  ->  tailscale up (OAuth)  ->  scp firmware.bin rpi  ->  ssh rpi 'sudo smarthome ota <kind> <plik>'
                                                                                          |
                                                              ESP pobiera z Pi po LAN  <--+
```

Wymagane sekrety repozytorium:

| Sekret | Opis |
| :----- | :--- |
| `TS_OAUTH_CLIENT_ID`, `TS_OAUTH_SECRET` | OAuth klienta Tailscale (lub jednorazowy `TS_AUTHKEY`) dla runnera Actions. |
| `RPI_SSH_HOST` | Nazwa Pi w tailnecie (MagicDNS, np. `rpi-smarthome`) lub jego IP `100.x.y.z`. |
| `RPI_SSH_USER` | Użytkownik SSH na Pi (musi móc `sudo smarthome` lub być w grupie `smarthome`). |
| `RPI_SSH_KEY` | Klucz prywatny SSH do logowania na Pi (albo użyj Tailscale SSH i pomiń ten sekret). |

Uruchomienie: zakładka **Actions → Firmware OTA → Run workflow**, wybierz moduł (`security`/`access`/
`environment`/`system`) i gałąź/commit. Workflow zbuduje właściwy profil i wypchnie OTA na zamontowaną makietę.

> Tailscale SSH (włączane na Pi przez setup: `tailscale up --ssh`) pozwala pominąć zarządzanie kluczami SSH –
> runner z odpowiednim tagiem ACL łączy się bez `RPI_SSH_KEY`.

---

## 5. Alternatywa lokalna (komputer dewelopera w tej samej sieci)

Gdy jesteś przy makiecie i nie chcesz angażować Pi jako hosta:

```bash
cd firmware/.pio/build/security_system/
python -m http.server 8000                       # serwuj firmware.bin z komputera
# w drugim terminalu ustal IP komputera (ipconfig / ip a), potem z Pi lub MQTT Explorer:
mosquitto_pub -h rpi-smarthome.local -u esp32 -P esp32 \
  -t home/security/update -m "http://<IP-KOMPUTERA>:8000/firmware.bin"
```

W oknie serwera HTTP zobaczysz `GET /firmware.bin ... 200`, a w logu płytki postęp z tagiem `OTA`.

---

## 6. Bezpieczeństwo operacji i rollback

- **Izolacja modułów:** komunikat odbiera tylko węzeł danego tematu; pozostałe ESP-ki działają bez zakłóceń.
- **Weryfikacja obrazu:** przed przełączeniem partycji ESP-IDF sprawdza poprawność obrazu. Błędny/niezgodny
  plik jest odrzucany (`esp_ota_abort`), a węzeł **pozostaje na dotychczasowej, sprawnej wersji** – patrz
  logika w `src/network_core.cpp` (`ota_update_task`).
- **Brak utraty łączności sterowania:** logika sprzętowa działa lokalnie; nawet nieudany OTA nie blokuje węzła.
- **Rollback:** wgraj ponownie poprzedni `firmware.bin` tą samą drogą. (Partycje OTA: `firmware/partitions.csv`.)

> Pełny kontrakt tematów (w tym `home/<kind>/update`): [MQTT_API.md](MQTT_API.md).
