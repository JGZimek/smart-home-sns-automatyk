#!/bin/bash
set -e

if [ "$EUID" -ne 0 ]; then
  echo "Uruchamiam z sudo..."
  exec sudo "$0" "$@"
fi

echo "[1/3] Konfiguracja WiFi Power Management..."
IWCONFIG_PATH=$(command -v iwconfig)
cat <<EOF > "/etc/systemd/system/wifi-power-off.service"
[Unit]
Description=Disable WiFi Power Management for stability
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=${IWCONFIG_PATH} wlan0 power off

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable wifi-power-off.service

echo "[2/3] Optymalizacja SWAP..."
SWAP_CONF="/etc/dphys-swapfile"
if command -v dphys-swapfile >/dev/null 2>&1; then
    echo "CONF_SWAPSIZE=1024" > "$SWAP_CONF"
    dphys-swapfile setup
    systemctl restart dphys-swapfile
fi

echo "[3/3] Konfiguracja Watchdog..."
SYSTEM_CONF="/etc/systemd/system.conf"
if ! grep -q '^RuntimeWatchdogSec=15\b' "$SYSTEM_CONF"; then
    echo "RuntimeWatchdogSec=15" >> "$SYSTEM_CONF"
fi
echo "Gotowe!"