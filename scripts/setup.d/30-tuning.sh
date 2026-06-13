#!/usr/bin/env bash
# Krok 30: tuning systemu pod stabilna prace 24/7 na Pi Zero 2W (Ubuntu Server).
# Zastepuje stary host_config.sh (ktory zakladal Raspberry Pi OS: dphys-swapfile/iwconfig).

# --- 1. Wi-Fi power save OFF (eliminuje losowe rozlaczenia brokera) ---
# Renderer-agnostycznie: oneshot z 'iw' dziala niezaleznie od networkd/NM i NIE
# przejmuje interfejsu (bezpieczne dla zdalnej instalacji przez Wi-Fi).
if write_if_changed /etc/systemd/system/wifi-powersave-off.service <<'EOF'
[Unit]
Description=Smart Home: wylacz Wi-Fi power save (stabilnosc polaczenia z brokerem)
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'for d in $(iw dev 2>/dev/null | sed -n "s/\\s*Interface //p"); do iw dev "$d" set power_save off || true; done'

[Install]
WantedBy=multi-user.target
EOF
then
  log "Konfiguruje wylaczenie Wi-Fi power save (iw)."
  systemctl daemon-reload
fi
systemctl enable --now wifi-powersave-off.service >/dev/null 2>&1 || warn "Nie udalo sie ustawic power save off (sprawdz 'iw dev')."

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

# --- 3. Sprzetowy watchdog (OPT-IN, domyslnie OFF) ---
# Na Zero 2W przy marginalnym zasilaniu watchdog zamienia chwilowy zawis/brownout
# w twardy reboot. Wlaczaj swiadomie dopiero gdy zasilanie jest pewne (WATCHDOG_ENABLE=1).
WDOG_FILE="/etc/systemd/system.conf.d/10-watchdog.conf"
if [ "${WATCHDOG_ENABLE:-0}" = "1" ]; then
  if write_if_changed "$WDOG_FILE" <<'EOF'
# Smart Home: sprzetowy watchdog – reboot gdy systemd przestanie go karmic.
[Manager]
RuntimeWatchdogSec=15s
RebootWatchdogSec=2min
EOF
  then
    log "Watchdog sprzetowy WLACZONY (wymaga reboota by w pelni zadzialal)."
    systemctl daemon-reexec || true
  else
    ok "Watchdog juz skonfigurowany."
  fi
else
  if [ -f "$WDOG_FILE" ]; then
    log "Watchdog wylaczony (WATCHDOG_ENABLE=0) – usuwam wczesniejsza konfiguracje."
    rm -f "$WDOG_FILE"
    systemctl daemon-reexec || true
  else
    ok "Watchdog wylaczony (WATCHDOG_ENABLE=0)."
  fi
fi
