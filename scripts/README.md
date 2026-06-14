# Serwer Smart Home na Raspberry Pi (plug-and-play)

> Część dokumentacji projektu [Smart Home](../README.md). Zobacz też: [uruchomienie całości](../GETTING_STARTED.md).
>
> Kompletny runbook (instalacja od zera, dostęp przez Tailscale, dashboard, zmiana sieci, troubleshooting): **[DEPLOYMENT.md](../DEPLOYMENT.md)**. Ten plik to referencja techniczna (struktura, CLI, usługi).

Kompletny, samonaprawiający się serwer makiety na **Raspberry Pi Zero 2W + Ubuntu Server**.
Jedno polecenie stawia broker MQTT, mDNS, fallbacki sieciowe, zdalną diagnostykę ESP-ek i zdalny
dostęp przez Tailscale. Wszystko jest **idempotentne** – instalator można puszczać wielokrotnie.

## Instalacja jednym poleceniem

Na świeżym Ubuntu Server (po połączeniu przez SSH):

```bash
curl -fsSL https://raw.githubusercontent.com/JGZimek/smart-home-sns-automatyk/develop/scripts/bootstrap.sh | sudo bash
```

To wszystko. Bootstrap klonuje repo do `/opt/smarthome` i uruchamia `setup.sh`, który:

1. instaluje Docker + pakiety (avahi, NetworkManager, mosquitto-clients, zram-tools, jq…),
2. ustawia hostname **`rpi-smarthome`** i włącza **mDNS** (ESP-ki znajdują brokera jako `rpi-smarthome.local`),
3. tuninguje system pod 24/7 na Zero 2W (Wi-Fi power-save off, **zram** swap, sprzętowy **watchdog**),
4. tworzy konto brokera **`smarthome`/`smarthome`** i uruchamia **Mosquitto** (Docker, porty 1883 + 9001 WS),
5. instaluje usługi: monitor węzłów + web status, awaryjny AP Wi-Fi, opcjonalny LCD, watchdog brokera,
6. instaluje **Tailscale** (`tailscale up --ssh`) do zdalnego dostępu.

Po instalacji sprawdź:

```bash
smarthome status
```

> **Tailscale** wymaga jednorazowego zalogowania – setup wypisze link, albo wklej pre-auth key do
> `TAILSCALE_AUTHKEY` w `/etc/smarthome/smarthome.env` i puść `sudo smarthome update` (pełne plug-and-play).

## Konfiguracja

Wszystkie nastawy w **`/etc/smarthome/smarthome.env`** (tworzony z [`smarthome.env.example`](smarthome.env.example),
setup nie nadpisuje Twoich zmian). Po edycji: `sudo smarthome update` lub restart usługi.

Najważniejsze: `BROKER_USER/PASS`, `WEB_PORT`, `EXPECTED_KINDS`, `AP_*` (awaryjny AP), `LCD_*`, `TAILSCALE_*`.

## CLI `smarthome` – sterowanie i diagnostyka (SSH/Tailscale)

```bash
smarthome status              # broker, usługi, temperatura, zram, IP, Tailscale, liczba węzłów
smarthome nodes               # tabela węzłów ESP + ich zdrowie (online/stale/offline/missing)
smarthome cmd <kind> <komenda>  # reboot | reset_wifi | identify | diag | ping  -> home/<kind>/cmd
smarthome ota <kind> <plik|url>   # zdalny OTA: plik .bin hostowany na Pi lub gotowy URL -> home/<kind>/update
smarthome set <topic> <payload>   # surowa publikacja (np. home/access/door/set OPEN)
smarthome watch [topic]       # podgląd ruchu MQTT (domyślnie home/#)
smarthome wifi <...>          # sieć Wi-Fi serwera: status|list|connect|forget|portal|migrate-nm (offline)
smarthome logs <usługa>       # monitor | wifi | lcd | health | broker
smarthome broker <up|down|restart|logs>
smarthome update              # git pull + ponowny setup
smarthome tailscale status    # przekazanie do tailscale
```

`kind` = `security` | `access` | `environment` | `system`.

## Zdalna diagnostyka węzłów ESP

Usługa **`smarthome-node-monitor`** subskrybuje `home/+/{info,availability,diag}`, buduje rejestr węzłów
(`/var/lib/smarthome/nodes.json`), wykrywa węzły `stale`/`offline`/`missing` i serwuje **dashboard**:

- **`http://rpi-smarthome.local:8080`** – kafelki z każdym węzłem (status, RSSI, uptime, ostatni diag) + karta serwera.
  Dostępny też przez Tailscale MagicDNS (np. z telefonu, bez bycia w tej samej sieci LAN).
- **`http://rpi-smarthome.local:8080/api/nodes`** – ten sam stan w JSON (dla backendu/skryptów).

Karta **serwera** ma przycisk **„Sprawdz i zainstaluj aktualizacje"** (`WEB_UPDATE_ENABLE`), a karta
**„Sieć Wi-Fi"** (`WEB_WIFI_ENABLE`, wymaga NetworkManagera) pozwala **skanować, przełączać sieć i wymuszać
portal** wprost z dashboardu. Obie operacje wykonuje root przez usługi `smarthome-update` / `smarthome-wifi-apply`
(web tylko pisze plik-żądanie). **Przełączaj sieć przez Tailscale** – zmiana zrywa dostęp przez bieżącą sieć.

Monitor publikuje też zdrowie samego Pi jako węzeł `home/system/server/*` – backend widzi serwer tak samo
jak płytki ESP.

## Zdalny OTA (hosting firmware na Pi)

Ten sam serwer HTTP hostuje obrazy firmware dla aktualizacji OTA: pliki z `/var/lib/smarthome/firmware/`
są serwowane pod **`http://rpi-smarthome.local:8080/firmware/<nazwa>`**. Dzięki temu OTA jest w pełni zdalny –
`smarthome ota <kind> <plik.bin>` kopiuje obraz, buduje URL po IP LAN Pi i publikuje go na `home/<kind>/update`,
a ESP pobiera firmware lokalnie z Pi. Pełna procedura (w tym GitHub Actions przez Tailscale):
[firmware/OTA_UPDATE.md](../firmware/OTA_UPDATE.md).

## Mechanizmy odpornościowe (fallbacki)

| Mechanizm | Co robi |
| :-------- | :------ |
| **Awaryjny AP Wi-Fi** (`smarthome-wifi-fallback`) | **Opt-in** (`AP_FALLBACK_ENABLE=1` + Wi-Fi pod NetworkManager: `sudo smarthome wifi migrate-nm` – patrz niżej). Gdy Pi straci połączenie z siecią na dłużej niż karencja, podnosi własny AP `SmartHome-Config` ze stroną do **wyboru sieci ze skanu** lub wpisania ręcznie (`http://10.42.0.1`). Po sukcesie kasuje AP. Portal na żądanie: `sudo smarthome wifi portal` lub przycisk w dashboardzie. |
| **Watchdog brokera** (`smarthome-health.timer`) | Co 2 min sprawdza port 1883; po 2 nieudanych próbach restartuje kontener brokera. |
| **Sprzętowy watchdog** (systemd) | Reboot Pi przy zawisie systemu. |
| **zram swap** | Kompresowany swap w RAM – zapas pamięci na Zero 2W bez zużywania karty SD. |
| **Docker `restart: unless-stopped`** | Broker wstaje sam po reboocie / awarii kontenera. |

### Wi-Fi pod NetworkManager (potrzebne tylko dla awaryjnego AP)

Ubuntu Server domyślnie zarządza siecią przez **systemd-networkd/netplan**, a nie NetworkManager.
Broker, mDNS, OTA, Tailscale, dashboard i watchdogi **działają niezależnie od tego** – instalator celowo
**nie** instaluje NetworkManagera (instalacja NM po Wi-Fi przejęłaby `wlan0` i zerwała połączenie).
Power-save wyłączamy renderer-agnostycznie przez `iw` (usługa `wifi-powersave-off`).

Awaryjny AP wymaga jednak NM. Migracja jest zautomatyzowana **jedną komendą** (pre-tworzy profil NM
z Twoimi danymi, żeby połączenie nie padło, i przełącza renderer):

```bash
# URUCHOM PRZEZ TAILSCALE (100.x) lub z konsoli HDMI – wlan0 na chwilę zerwie!
sudo smarthome wifi migrate-nm "<TWOJ_SSID>" "<HASLO>"

# następnie włącz fallback w /etc/smarthome/smarthome.env:  AP_FALLBACK_ENABLE=1
sudo smarthome update
smarthome status        # powinno pokazać smarthome-wifi-fallback active
```

`migrate-nm` instaluje NetworkManagera, tworzy profil bieżącej sieci, ustawia `renderer: NetworkManager`
w netplan i weryfikuje, czy `wlan0` jest „managed". Awaryjny powrót (z konsoli):
`sudo rm /etc/netplan/99-networkmanager.yaml && sudo netplan apply`.

Po migracji działają też: `sudo smarthome wifi connect "SSID" "HASLO"` (przepięcie sieci) oraz
`sudo smarthome wifi portal` (wymuszony AP konfiguracyjny na żądanie).

## Struktura katalogu

```
scripts/
  bootstrap.sh          one-liner: klon repo -> setup
  setup.sh              idempotentny instalator (orkiestruje setup.d/*)
  setup.d/              kroki: 10-packages, 20-hostname-mdns, 30-tuning, 40-broker, 50-app-services, 60-tailscale
  lib/common.sh         wspólne helpery (logi, config, compose)
  smarthome.env.example wzorzec konfiguracji -> /etc/smarthome/smarthome.env
  bin/smarthome         CLI (symlink w /usr/local/bin)
  services/             node_monitor.py, wifi_fallback.py, lcd_display.py
  systemd/              jednostki usług + timer
  health-check.sh       watchdog brokera (z timera)
  requirements.txt      zależności Pythona (venv: scripts/.venv)
```

## Diagnozowanie problemów

```bash
smarthome status                       # ogólny obraz
smarthome logs monitor                 # logi monitora węzłów
smarthome logs broker                  # logi Mosquitto
journalctl -u smarthome-wifi-fallback  # awaryjny AP
ping rpi-smarthome.local               # czy mDNS działa (z innego urządzenia)
```

| Objaw | Przyczyna / rozwiązanie |
| :---- | :---------------------- |
| ESP-ki nie łączą się | mDNS: `ping rpi-smarthome.local`. Broker: `smarthome status`. Sieć **2.4 GHz**. |
| Broker odrzuca połączenia | Konto: setup tworzy `mosquitto/config/passwd` (`smarthome`/`smarthome`). `smarthome logs broker`. |
| Brak `nodes.json` / dashboardu | `systemctl status smarthome-node-monitor`, `smarthome logs monitor`. |
| AP `SmartHome-Config` nie wstaje | `journalctl -u smarthome-wifi-fallback`; wymaga NetworkManager + `wlan0`. |
| Tailscale offline | `smarthome tailscale status`; zaloguj: `sudo tailscale up --ssh`. |
