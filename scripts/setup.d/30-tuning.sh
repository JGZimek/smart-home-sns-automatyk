#!/usr/bin/env bash
# Krok 30: tuning systemu pod stabilna prace 24/7 na Pi Zero 2W (Ubuntu Server).
# Zastepuje stary host_config.sh (ktory zakladal Raspberry Pi OS: dphys-swapfile/iwconfig).

# --- 1. Wi-Fi power save OFF (eliminuje losowe rozlaczenia brokera) ---
# Na Ubuntu Wi-Fi obsluguje NetworkManager => konfigurujemy go, nie iwconfig.
if write_if_changed /etc/NetworkManager/conf.d/10-wifi-powersave-off.conf <<'EOF'
# Smart Home: wylacz oszczedzanie energii Wi-Fi (stabilnosc polaczenia z brokerem).
# 2 = disable power save.
[connection]
wifi.powersave = 2
EOF
then
  log "Wi-Fi power save wylaczony – restartuje NetworkManager."
  systemctl restart NetworkManager || warn "Nie udalo sie zrestartowac NetworkManager (moze byc OK przy pierwszej instalacji)."
else
  ok "Wi-Fi power save juz skonfigurowany."
fi

# --- 2. zram swap (kompresja RAM zamiast dphys-swapfile; idealne dla 512MB) ---
# zram-tools czyta /etc/default/zramswap. Ustawiamy ~50% RAM jako skompresowany swap.
if write_if_changed /etc/default/zramswap <<'EOF'
# Smart Home: kompresowany swap w RAM (oszczedza karte SD, daje zapas pamieci).
ALGO=zstd
PERCENT=50
PRIORITY=100
EOF
then
  log "Konfiguruje zram swap (50% RAM, zstd)."
  systemctl enable --now zramswap.service >/dev/null 2>&1 || systemctl restart zramswap.service || warn "zramswap nie wstal – sprawdz 'systemctl status zramswap'."
else
  ok "zram swap juz skonfigurowany."
fi

# --- 3. Sprzetowy watchdog (auto-reboot przy zawisie OS; bcm2835_wdt na RPi) ---
if write_if_changed /etc/systemd/system.conf.d/10-watchdog.conf <<'EOF'
# Smart Home: sprzetowy watchdog – reboot gdy systemd przestanie go karmic.
[Manager]
RuntimeWatchdogSec=15s
RebootWatchdogSec=2min
EOF
then
  log "Watchdog sprzetowy skonfigurowany (wymaga reboota by w pelni zadzialal)."
  systemctl daemon-reexec || true
else
  ok "Watchdog juz skonfigurowany."
fi
