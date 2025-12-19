#!/bin/bash

# --- KONFIGURACJA ---
# Ustalanie ścieżek bezwzględnych (ważne dla systemd)
SCRIPT_DIR=$(pwd)
VENV_DIR="$SCRIPT_DIR/lcd_env"
PYTHON_SCRIPT="$SCRIPT_DIR/broker_lcd.py"
SERVICE_NAME="broker_lcd"
USER_NAME=$(whoami)
I2C_ADDR=0x27  # Zmień na 0x3f jeśli ekran nie zadziała
# --------------------

echo ">>> [1/6] Aktualizacja i instalacja zależności systemowych..."
sudo apt update
sudo apt install -y i2c-tools python3-pip python3-venv

# Dodanie użytkownika do grupy i2c (aby działało bez sudo w przyszłości)
sudo usermod -aG i2c $USER_NAME

echo ">>> [2/6] Tworzenie lekkiego środowiska wirtualnego (Ubuntu Noble)..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv $VENV_DIR
fi

echo ">>> [3/6] Instalacja bibliotek w venv..."
# Instalujemy w izolowanym środowisku
$VENV_DIR/bin/pip install RPLCD smbus2

echo ">>> [4/6] Tworzenie zoptymalizowanego skryptu Python..."
cat << EOF > $PYTHON_SCRIPT
import time
import socket
import fcntl
import struct
import sys
from RPLCD.i2c import CharLCD

# --- KONFIGURACJA ---
I2C_ADDRESS = $I2C_ADDR
CHECK_INTERVAL = 60  # Sprawdzaj IP co 60 sekund (oszczędność CPU)

def get_ip_address(ifname):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode('utf-8'))
        )[20:24])
    except:
        return None

def main():
    try:
        # Inicjalizacja LCD
        lcd = CharLCD(i2c_expander='PCF8574', address=I2C_ADDRESS, port=1,
                      cols=16, rows=2, dotsize=8,
                      charmap='A00',
                      auto_linebreaks=True,
                      backlight_enabled=True)
    except Exception as e:
        print(f"Błąd inicjalizacji LCD: {e}")
        sys.exit(1)

    lcd.clear()
    lcd.write_string('Bootowanie...')
    
    last_ip = ""

    while True:
        # Priorytet: WiFi (wlan0), potem kabel (eth0)
        current_ip = get_ip_address('wlan0')
        if not current_ip:
             current_ip = get_ip_address('eth0')
        
        if not current_ip:
            current_ip = "Brak sieci..."

        # Rysuj na ekranie TYLKO jeśli IP się zmieniło (oszczędność I2C i CPU)
        if current_ip != last_ip:
            try:
                lcd.clear()
                lcd.write_string('MQTT Broker IP:')
                lcd.cursor_pos = (1, 0)
                lcd.write_string(current_ip)
                last_ip = current_ip
            except Exception as e:
                print(f"Błąd I2C: {e}")

        # RPi Zero 2W może spać. To nie blokuje systemu.
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
EOF

echo ">>> [5/6] Tworzenie pliku usługi Systemd..."
# Tworzymy plik .service dynamicznie podstawiając ścieżki
sudo bash -c "cat << EOF > /etc/systemd/system/${SERVICE_NAME}.service
[Unit]
Description=LCD Display Service for SmartHome Broker
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$SCRIPT_DIR
# Uruchamiamy pythona z wewnątrz venv
ExecStart=$VENV_DIR/bin/python $PYTHON_SCRIPT
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF"

echo ">>> [6/6] Uruchamianie usługi..."
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
sudo systemctl restart $SERVICE_NAME

echo "-------------------------------------------------------"
echo "SUKCES! Usługa '$SERVICE_NAME' została zainstalowana."
echo "Status sprawdzisz komendą: systemctl status $SERVICE_NAME"
echo "Logi sprawdzisz komendą: journalctl -u $SERVICE_NAME -f"
echo "-------------------------------------------------------"
