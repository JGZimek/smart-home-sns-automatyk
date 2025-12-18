#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Error: Please run this script as root (sudo)."
  exit 1
fi

echo "=== Starting Raspberry Pi Zero 2 W Optimization for SmartHome ==="

# --- 1. WIFI POWER MANAGEMENT (Disable for stability) ---
echo "[1/3] Configuring WiFi Power Management..."

# Verify if iwconfig is available BEFORE creating the service
if ! command -v iwconfig >/dev/null 2>&1; then
    echo "Error: 'iwconfig' command not found."
    echo "Please install wireless-tools (sudo apt install wireless-tools) and retry."
    exit 1
fi

SERVICE_FILE="/etc/systemd/system/wifi-power-off.service"

# Create service file
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

echo "   -> Service file created."

# Reload and enable service
systemctl daemon-reload
systemctl enable wifi-power-off.service
systemctl start wifi-power-off.service

# Verification
CURRENT_PM=$(iwconfig wlan0 2>/dev/null | grep "Power Management" || echo "Unknown")
echo "   -> Current Status: $CURRENT_PM"


# --- 2. SWAP CONFIGURATION (Increase to 1024MB) ---
SWAP_CONF="/etc/dphys-swapfile"
echo "[2/3] Increasing SWAP size to 1024MB..."

if ! command -v dphys-swapfile >/dev/null 2>&1; then
    echo "Warning: 'dphys-swapfile' is not installed. Skipping SWAP optimization."
    echo "Install it via 'sudo apt install dphys-swapfile' if needed."
else
    if [ ! -f "$SWAP_CONF" ]; then
        echo "   -> Configuration file $SWAP_CONF does not exist. Creating new one..."
        echo "CONF_SWAPSIZE=1024" > "$SWAP_CONF"
        
        dphys-swapfile setup
        dphys-swapfile swapon
        echo "   -> Swap set to 1024MB (new file created)."
    else
        # CREATE BACKUP
        cp "$SWAP_CONF" "${SWAP_CONF}.bak"
        echo "   -> Backup created at ${SWAP_CONF}.bak"

        # Universal logic: find CONF_SWAPSIZE (commented or not) and enforce 1024
        # We use grep in if condition (it's safe with set -e)
        if grep -qE "^#?CONF_SWAPSIZE=" "$SWAP_CONF"; then
            sed -i -E 's/^#?CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' "$SWAP_CONF"
            echo "   -> Configuration updated in file."
        else
            echo "CONF_SWAPSIZE=1024" >> "$SWAP_CONF"
            echo "   -> Configuration appended to file."
        fi
        
        # Apply changes
        dphys-swapfile setup
        dphys-swapfile swapon
        echo "   -> Swap service restarted successfully."
    fi
fi


# --- 3. HARDWARE WATCHDOG (Auto-reboot on freeze) ---
SYSTEM_CONF="/etc/systemd/system.conf"
echo "[3/3] Configuring Systemd RuntimeWatchdog..."

# CREATE BACKUP
cp "$SYSTEM_CONF" "${SYSTEM_CONF}.bak"
echo "   -> Backup created at ${SYSTEM_CONF}.bak"

if grep -q '^#RuntimeWatchdogSec=[[:space:]]*$' "$SYSTEM_CONF"; then
    sed -i 's/^#RuntimeWatchdogSec=[[:space:]]*$/RuntimeWatchdogSec=15/' "$SYSTEM_CONF"
    echo "   -> Watchdog set to 15s."
elif grep -q "RuntimeWatchdogSec=15" "$SYSTEM_CONF"; then
    echo "   -> Watchdog is already configured correctly."
else
    if grep -q "RuntimeWatchdogSec=" "$SYSTEM_CONF"; then
         echo "   -> Watchdog is set to a non-default value. Skipping modification."
    else
         echo "Warning: Standard RuntimeWatchdogSec line not found in $SYSTEM_CONF. Please check manually."
    fi
fi

echo "=== Optimization Complete. Recommended action: sudo reboot ==="
