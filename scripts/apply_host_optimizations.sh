#!/bin/bash

# Upewnij się, że skrypt jest uruchomiony jako root
if [ "$EUID" -ne 0 ]
  then echo "Proszę uruchom ten skrypt jako root (sudo)"
  exit
fi

echo "=== Rozpoczynam optymalizację Raspberry Pi Zero 2 W dla SmartHome ==="

# --- 1. WYŁĄCZENIE OSZCZĘDZANIA ENERGII WIFI (Power Management) ---
# Tworzymy usługę systemd, która przy każdym starcie wyłączy oszczędzanie energii.
# Jest to pewniejsze niż rc.local.

SERVICE_FILE="/etc/systemd/system/wifi-power-off.service"

echo "[1/3] Konfiguracja WiFi Power Management..."

cat <<EOF > $SERVICE_FILE
[Unit]
Description=Disable WiFi Power Management for stability
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iwconfig wlan0 power off

[Install]
WantedBy=multi-user.target
EOF

# Przeładowanie systemd i włączenie usługi
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
# Zwiększenie z domyślnych 100MB na 1024MB, aby Docker nie zabijał procesów.

SWAP_CONF="/etc/dphys-swapfile"
echo "[2/3] Zwiększanie SWAP do 1024MB..."

if ! command -v dphys-swapfile >/dev/null 2>&1; then
    echo "   -> Polecenie 'dphys-swapfile' nie jest dostępne. Pomijam zmianę SWAP. Zainstaluj pakiet 'dphys-swapfile', aby włączyć tę optymalizację."
elif [ ! -f "$SWAP_CONF" ]; then
    echo "   -> Plik konfiguracyjny $SWAP_CONF nie istnieje. Pomijam zmianę SWAP."
else
    if grep -q "CONF_SWAPSIZE=100" "$SWAP_CONF"; then
        # Zmiana 100 na 1024
        sed -i 's/^CONF_SWAPSIZE=100$/CONF_SWAPSIZE=1024/' "$SWAP_CONF"
        
        # Zastosowanie zmian
        dphys-swapfile setup
        dphys-swapfile swapon
        echo "   -> Swap zwiększony pomyślnie."
    else
        echo "   -> Wygląda na to, że Swap jest już zmieniony lub plik konfiguracyjny jest inny."
    fi
fi


# --- 3. AKTYWACJA WATCHDOG ---
# Restartuje system, jeśli zawiesi się na dłużej niż 15 sekund.

SYSTEM_CONF="/etc/systemd/system.conf"
echo "[3/3] Konfiguracja Systemd RuntimeWatchdog..."

# Odkomentowanie i ustawienie RuntimeWatchdogSec (jeśli jest zakomentowane)
if grep -q "#RuntimeWatchdogSec=" "$SYSTEM_CONF"; then
    sed -i 's/^#RuntimeWatchdogSec=.*$/RuntimeWatchdogSec=15/' "$SYSTEM_CONF"
    echo "   -> Watchdog ustawiony na 15s."
elif grep -q "RuntimeWatchdogSec=15" "$SYSTEM_CONF"; then
    echo "   -> Watchdog już jest skonfigurowany."
else
    # Jeśli linia nie istnieje lub jest inna, dopisujemy na koniec (mniej eleganckie, ale skuteczne)
    # Dla bezpieczeństwa prosta informacja dla użytkownika
    echo "   -> Nie znaleziono standardowej linii w \"$SYSTEM_CONF\". Sprawdź plik ręcznie."
fi

echo "=== Zakończono. Zalecany restart systemu: sudo reboot ==="
