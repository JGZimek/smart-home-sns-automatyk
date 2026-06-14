# Wdrożenie i obsługa serwera RPi (kompletny runbook)

> 📦 Część dokumentacji projektu [Smart Home](README.md). Zobacz też: [uruchomienie całości](GETTING_STARTED.md) · [serwer – referencja](scripts/README.md) · [kontrakt MQTT](firmware/MQTT_API.md) · [OTA](firmware/OTA_UPDATE.md).

Ten dokument prowadzi przez **wszystko** dotyczące Raspberry Pi (serwera makiety):
1. pierwsze uruchomienie od pustej karty,
2. jak dostać się do działającego serwera (SSH, dashboard),
3. jak przekonfigurować na inną sieć Wi-Fi,
4. konfigurację, aktualizację i rozwiązywanie problemów.

Cel: po jednorazowej konfiguracji serwer jest **plug-and-play** — broker MQTT `rpi-smarthome.local:1883`,
mDNS, diagnostyka węzłów + dashboard, zdalny dostęp przez Tailscale.

---

## 1. Pierwsze uruchomienie (od pustej karty microSD)

Potrzebne: karta microSD + czytnik, Raspberry Pi Imager, konto Tailscale, (później) telefon z aplikacją
**ESP BLE Provisioning**.

### 1.1. Klucz Tailscale (PC, przeglądarka)
[Tailscale → Settings → Keys](https://login.tailscale.com/admin/settings/keys) → **Generate auth key**
(Ephemeral: OFF). Skopiuj `tskey-auth-...`.

### 1.2. Wgranie systemu (Raspberry Pi Imager)
- Device: **Raspberry Pi Zero 2 W**, OS: **Ubuntu Server 24.04 LTS (64-bit)**, Storage: karta.
- **Edit Settings → General:** hostname `rpi-smarthome`; użytkownik `smarthome` + hasło; **Wi-Fi** SSID+hasło
  **2.4 GHz** + kraj **PL**; locale Europe/Warsaw.
- **Services:** Enable SSH → password authentication.
- **Save → Write.** Po zapisie nie wyjmuj karty.

### 1.3. Tailscale na pierwszy boot (PC) — zalecane
Dzięki temu wejdziesz na Pi bez znajomości IP i mimo izolacji klientów w sieci (hotspoty często izolują).
Otwórz partycję **`system-boot`** → plik **`user-data`** → dopisz na końcu (2 spacje wcięcia, **bez tabów**):
```yaml
runcmd:
  - [ sh, -c, "curl -fsSL https://tailscale.com/install.sh | sh" ]
  - [ tailscale, up, "--ssh", "--hostname=rpi-smarthome", "--authkey=WKLEJ_TU_KLUCZ" ]
```
Jeśli `runcmd:` już istnieje — dopisz tylko dwie linie `- [...]` pod nim. W `--authkey=` wstaw klucz
skopiowany w §1.1 (zaczyna się od `tskey-auth-`). Zapisz, wysuń kartę.

> **Bez Tailscale prebake:** alternatywnie znajdź IP Pi skanem LAN (`arp -a` + MAC `b8-27-eb|dc-a6-32|e4-5f-01|2c-cf-67`) — ale działa tylko w sieci bez izolacji klientów.

### 1.4. Boot i wejście na Pi
Włóż kartę, zasil, odczekaj **~5 min** (boot → Wi-Fi → instalacja Tailscale + pierwsze auto-aktualizacje).
```powershell
tailscale status              # [PC] zobaczysz rpi-smarthome i adres 100.x.y.z
ssh smarthome@rpi-smarthome   # lub ssh smarthome@100.x.y.z
```

### 1.5. Szybki sprawdzian łączności (na Pi, opcjonalnie)
```bash
ping -c2 1.1.1.1                                         # internet
getent hosts raw.githubusercontent.com && echo "DNS OK" # rozwiazywanie nazw
```

### 1.6. Instalacja serwera (jedno polecenie, w tmux)
```bash
sudo apt-get install -y tmux
tmux new -s setup
curl -fsSL https://raw.githubusercontent.com/JGZimek/smart-home-sns-automatyk/main/scripts/bootstrap.sh | sudo bash
```
Instalator (idempotentny) stawia: Docker + broker MQTT, hostname/mDNS, konto brokera **`smarthome`/`smarthome`**,
zram, wyłączenie Wi-Fi power-save, monitor węzłów + dashboard, Tailscale. Watchdog sprzętowy i awaryjny AP Wi-Fi
są **domyślnie wyłączone** (opt-in). `tmux` zabezpiecza instalację przed rozłączeniem SSH (`tmux attach -t setup`).

### 1.7. Weryfikacja
```bash
smarthome status
```
Oczekiwane: Broker `UP`, Port 1883 `nasluchuje`, usługi `active`, Tailscale `100.x.y.z`, „Wezly ESP: 0/3".

✅ **Serwer gotowy.** Wgranie firmware i provisioning ESP-ek: [GETTING_STARTED.md](GETTING_STARTED.md#2-moduły-esp32-firmware).

---

## 2. Dostęp do działającego serwera

| Co | Jak |
| :- | :-- |
| **SSH (z dowolnego miejsca)** | `ssh smarthome@rpi-smarthome` (MagicDNS) lub `ssh smarthome@100.x.y.z` (Tailscale). |
| **SSH w tej samej LAN** | `ssh smarthome@rpi-smarthome.local` (mDNS) — gdy sieć nie izoluje klientów. |
| **Dashboard (przeglądarka)** | `http://100.x.y.z:8080` (Tailscale, zawsze) · `http://rpi-smarthome:8080` (MagicDNS) · `http://rpi-smarthome.local:8080` (LAN). |
| **API JSON** | `http://<host>:8080/api/nodes` · health: `/healthz`. |
| **CLI** | `smarthome status` / `nodes` / `watch` / `logs` / `cmd` / `ota` (patrz [scripts/README.md](scripts/README.md)). |

> **Adres Tailscale `100.x.y.z` jest stały** niezależnie od sieci — używaj go, bo lokalne IP Pi zmienia się
> przy przenoszeniu makiety między sieciami.

### Ładny link HTTPS w tailnecie (opcjonalnie)
```bash
sudo tailscale serve --bg 8080      # wystawia dashboard jako https://rpi-smarthome.<tailnet>.ts.net (tylko w tailnecie)
tailscale serve status              # pokaze URL
# wylaczenie: sudo tailscale serve --https=443 off
```

---

## 3. Przekonfigurowanie na inną sieć Wi-Fi

Makietę przewozisz w inne miejsce / zmieniasz router. Trzeba ustawić **i Pi, i ESP-ki** na nową sieć.

### 3.1. Raspberry Pi → nowa sieć

**Z dashboardu** (`http://rpi-smarthome.local:8080` lub przez Tailscale): karta **„Sieć Wi-Fi"** — skan,
wybór sieci, hasło, „Połącz", oraz przyciski „Portal"/„Skanuj". Wymaga NetworkManagera (`migrate-nm`, niżej).
Przełączaj **przez Tailscale**, bo zmiana zrywa dostęp przez bieżącą sieć.

**Z CLI** — poleceniem **`smarthome wifi`** (działa lokalnie, **bez internetu** — z konsoli/HDMI lub SSH po LAN/Tailscale):

```bash
smarthome wifi                              # status: aktywna siec, IP, zapisane sieci
smarthome wifi list                         # skan sieci w zasiegu
sudo smarthome wifi connect "<SSID>" "<HASLO>"   # przepiecie na nowa siec
sudo smarthome wifi forget "<SSID>"         # usuniecie zapisanej sieci
```

**Nie masz dostępu do Pi (brak konsoli/SSH) i brak internetu?** Wymuś portal konfiguracyjny z telefonu:

```bash
sudo smarthome wifi portal                  # podnosi AP 'SmartHome-Config' na zadanie
#  -> polacz telefon z AP, otworz http://10.42.0.1, podaj nowa siec
sudo smarthome wifi portal off              # anuluj (powrot do zapisanej sieci)
```

Portal na żądanie działa nawet, gdy Pi jest jeszcze połączone ze starą siecią — w odróżnieniu od
automatycznego AP fallback (usługa `smarthome-wifi-fallback`), który podnosi się dopiero po **utracie**
łączności.

**Wymóg:** `connect`/`portal`/AP fallback działają tylko gdy Wi-Fi jest pod **NetworkManagerem**
(Ubuntu Server używa networkd). Przełączenie następuje **po reboocie** — komenda niczego nie zrywa na żywo:

```bash
sudo smarthome wifi migrate-nm "<TWOJ_SSID>" "<HASLO>"   # przygotowanie (bez zrywania sieci)
sudo reboot                                              # MIEJ KONSOLĘ HDMI – po reboocie NM przejmuje wlan0
# po reboocie:
sudo sed -i 's/^AP_FALLBACK_ENABLE=.*/AP_FALLBACK_ENABLE=1/' /etc/smarthome/smarthome.env
sudo smarthome update
```
⚠️ Jeśli po reboocie NM nie połączy z Wi-Fi, **Tailscale też padnie** — stąd wymóg konsoli HDMI przy
pierwszej migracji. Powrót: `sudo rm /etc/netplan/99-networkmanager.yaml /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg && sudo netplan apply`.

- Alternatywnie ręcznie: `sudo nmcli dev wifi connect "<SSID>" password "<HASLO>"`.
- Tailscale i konto brokera nie wymagają zmian. mDNS (`rpi-smarthome.local`) działa dalej po połączeniu.

**Straciłeś dostęp do Pi?** Kolejność ratunku:
1. Pi **bez żadnej sieci** → po ~90 s само stawia AP `SmartHome-Config` (jeśli AP fallback włączony) → telefon → `http://10.42.0.1`.
2. Pi **ma internet** w innej sieci → wejdź przez Tailscale `ssh smarthome@100.x` → `sudo smarthome wifi connect ...`.
3. Ostateczność → konsola HDMI/klawiatura lub edycja sieci na karcie SD.

### 3.2. ESP-ki → nowa sieć
Poświadczenia Wi-Fi w ESP są w pamięci NVS. Aby je zmienić:
1. **Zdalnie wyzwól ponowne parowanie** (gdy moduł jest jeszcze online na starej sieci):
   ```bash
   smarthome cmd security reset_wifi      # oraz access / environment
   ```
   Moduł skasuje sieć i wróci do trybu BLE. (Jeśli już stracił sieć — zadziała auto-fallback po 5 próbach.)
2. **Telefonem (ESP BLE Provisioning)** podaj nową sieć 2.4 GHz — jak przy pierwszym uruchomieniu
   ([WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md), PoP `smarthome`).

### 3.3. „Zero-touch" na targi (bez przekonfigurowania w terenie)
Sprovisionuj ESP-ki **raz** do sieci o stałym SSID/haśle, którą zapewnia samo Pi (Pi jako AP). Wtedy makieta
działa wszędzie bez internetu i bez re-provisioningu. Szczegóły: sekcja „Wi-Fi pod NetworkManager"
w [scripts/README.md](scripts/README.md) + [WIFI_PROVISIONING.md](firmware/WIFI_PROVISIONING.md) §6.2.

---

## 4. Konfiguracja (`/etc/smarthome/smarthome.env`)

Wszystkie nastawy w jednym pliku. Po edycji: `sudo smarthome update` (lub restart usługi).

| Klucz | Domyślnie | Znaczenie |
| :---- | :-------- | :-------- |
| `BROKER_USER` / `BROKER_PASS` | `smarthome` / `smarthome` | Konto brokera (zsynchronizowane z firmware `module_config.h`). |
| `WEB_PORT` | `8080` | Port dashboardu / API. |
| `EXPECTED_KINDS` | `security,access,environment` | Węzły oczekiwane (brak = „MISSING" w dashboardzie). |
| `DIAG_STALE_S` | `90` | Po ilu s bez heartbeatu węzeł = „stale". |
| `WATCHDOG_ENABLE` | `0` | Sprzętowy watchdog (auto-reboot przy zawisie OS). Włącz, gdy masz pewne zasilanie 5 V/3 A. |
| `AP_FALLBACK_ENABLE` | `0` | Awaryjny AP Wi-Fi. Wymaga Wi-Fi pod NetworkManagerem (patrz [scripts/README.md](scripts/README.md)). |
| `TAILSCALE_ENABLE` / `TAILSCALE_SSH` | `1` / `1` | Tailscale + Tailscale SSH. |
| `TAILSCALE_TAGS` | (puste) | Np. `tag:smarthome` — przydatne dla OTA z GitHub Actions. |

---

## 5. Aktualizacja i OTA
- **Serwer (CLI):** `sudo smarthome update` (git pull + ponowny setup + restart usług, idempotentnie).
  Pi **nie aktualizuje się samo** — to świadoma akcja.
- **Serwer (z dashboardu):** przycisk **„Sprawdz i zainstaluj aktualizacje"** na karcie serwera
  (`http://rpi-smarthome.local:8080`). Web (user `smarthome`) tworzy plik-wyzwalacz, a rootowa usługa
  `smarthome-update` wykonuje to samo co `smarthome update`. Wynik (wersja, „nowa/bez zmian") pojawia się
  na karcie serwera po odświeżeniu. Wyłącznik: `WEB_UPDATE_ENABLE=0` (gdy dashboard wystawiony poza zaufany LAN/Tailscale).
- **Firmware ESP (zdalnie):** `smarthome ota <kind> <plik.bin>` lub z GitHub Actions — patrz
  [firmware/OTA_UPDATE.md](firmware/OTA_UPDATE.md).

---

## 6. Rozwiązywanie problemów

| Objaw | Przyczyna / rozwiązanie |
| :---- | :---------------------- |
| Pi **resetuje się** pod obciążeniem | Niedobór zasilania (Zero 2W jest na to czuły). Użyj zasilacza **5 V/3 A** i dobrego, krótkiego kabla; nie zasilaj z portu USB laptopa/huba. Sprawdź: `sudo journalctl -k -b -1 \| grep -i voltage`. |
| `ping rpi-smarthome.local` nie działa | mDNS działa w tej samej sieci LAN (bez izolacji klientów). Z innej sieci łącz się przez Tailscale (`100.x.y.z`). |
| ESP nie pojawia się w `smarthome nodes` | Musi być na **tej samej** sieci 2.4 GHz co Pi; firmware zbudowane z aktualnego repo (konto `smarthome/smarthome`). Podejrzyj ruch: `smarthome watch 'home/#'`. |
| Tailscale offline po przeniesieniu makiety | Nowa sieć musi mieć internet, by Pi wróciło do tailnetu. Sprawdź `sudo tailscale status`. |
| Dashboard nie otwiera się | `smarthome status` (czy `smarthome-node-monitor` active); z innej sieci użyj adresu Tailscale `http://100.x.y.z:8080`. |

Logi usług: `smarthome logs monitor` / `broker` / `wifi` / `lcd` / `health`.
