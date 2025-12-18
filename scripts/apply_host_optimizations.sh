#!/bin/bash

# Upewnij się, że skrypt jest uruchomiony jako root
if [ "$EUID" -ne 0 ]
  then echo "Proszę uruchom ten skrypt jako root (sudo)"
  exit 1
fi

echo "=== Rozpoczynam optymalizację Raspberry Pi Zero 2 W dla SmartHome ==="

# --- 1. WYŁĄCZENIE OSZCZĘDZANIA ENERGII WIFI (Power Management) ---
SERVICE_FILE="/etc/systemd/system/wifi-power-off.service"

echo "[1/3] Konfiguracja WiFi Power Management..."

cat <<EOF > "$SERVICE_FILE"
[Unit]
Description=Disable WiFi Power Management for stability
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iwconfig wlan0 power off

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable wifi-power-off.service
systemctl start wifi-power-off.service

# Weryfikacja
if command -v iwconfig >/dev/null 2>&1; then
    CURRENT_PM=$(iwconfig wlan0 2>/dev/null | grep "Power Management" || echo "Power Management status unavailable")
else
    CURRENT_PM="iwconfig command not found; cannot verify Power Management status"
fi
echo "   -> Status: $CURRENT_PM"


# --- 2. ZWIĘKSZENIE PLIKU WYMIANY (SWAP) ---
SWAP_CONF="/etc/dphys-swapfile"
echo "[2/3] Zwiększanie SWAP do 1024MB..."

if ! command -v dphys-swapfile >/dev/null 2>&1; then
    echo "   -> Polecenie 'dphys-swapfile' nie jest dostępne. Pomijam zmianę SWAP. Zainstaluj pakiet 'dphys-swapfile' (sudo apt install dphys-swapfile)."
elif [ ! -f "$SWAP_CONF" ]; then
    echo "   -> Plik konfiguracyjny $SWAP_CONF nie istnieje. Tworzę nowy..."
    echo "CONF_SWAPSIZE=1024" > "$SWAP_CONF"
    dphys-swapfile setup
    dphys-swapfile swapon
    echo "   -> Swap ustawiony na 1024MB."
else
    # Logika uniwersalna: szuka CONF_SWAPSIZE (zakomentowanego lub nie) i wymusza 1024
    if grep -qE "^#?CONF_SWAPSIZE=" "$SWAP_CONF"; then
        sed -i -E 's/^#?CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' "$SWAP_CONF"
        echo "   -> Zaktualizowano konfigurację w pliku."
    else
        echo "CONF_SWAPSIZE=1024" >> "$SWAP_CONF"
        echo "   -> Dodano konfigurację do pliku."
    fi
    
    # Zastosowanie zmian
    dphys-swapfile setup
    dphys-swapfile swapon
    echo "   -> Restart usługi swap wykonany."
fi


# --- 3. AKTYWACJA WATCHDOG ---
SYSTEM_CONF="/etc/systemd/system.conf"
echo "[3/3] Konfiguracja Systemd RuntimeWatchdog..."

if grep -q '^#RuntimeWatchdogSec=[[:space:]]*$' "$SYSTEM_CONF"; then
    sed -i 's/^#RuntimeWatchdogSec=[[:space:]]*$/RuntimeWatchdogSec=15/' "$SYSTEM_CONF"
    echo "   -> Watchdog ustawiony na 15s."
elif grep -q "RuntimeWatchdogSec=15" "$SYSTEM_CONF"; then
    echo "   -> Watchdog już jest skonfigurowany."
else
    # Jeśli linia jest inna lub jej brak, sprawdźmy czy przypadkiem nie jest ustawiona na inną wartość
    if grep -q "RuntimeWatchdogSec=" "$SYSTEM_CONF"; then
         echo "   -> Watchdog jest ustawiony na inną wartość niż domyślna. Nie zmieniam."
    else
         echo "   -> Nie znaleziono standardowej linii w \"$SYSTEM_CONF\". Sprawdź plik ręcznie."
    fi
fi

echo "=== Zakończono. Zalecany restart systemu: sudo reboot ==="
