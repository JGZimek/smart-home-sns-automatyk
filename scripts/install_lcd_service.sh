#!/bin/bash

# --- CONFIGURATION ---
# Setting absolute paths (important for systemd)
SCRIPT_DIR=$(pwd)
VENV_DIR="$SCRIPT_DIR/lcd_env"
PYTHON_SCRIPT="$SCRIPT_DIR/broker_lcd.py"
SERVICE_NAME="broker_lcd"
USER_NAME=$(whoami)
I2C_ADDR=0x27  # Change to 0x3f if screen doesn't work
# --------------------

echo ">>> [1/6] Updating system and installing system dependencies..."
sudo apt update
sudo apt install -y i2c-tools python3-pip python3-venv

# Add user to i2c group (for interactive use)
sudo usermod -aG i2c $USER_NAME

echo ">>> [2/6] Creating lightweight virtual environment (Ubuntu Noble)..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv $VENV_DIR
fi

echo ">>> [3/6] Installing libraries in venv..."
# Installing in isolated environment with pinned versions for security
$VENV_DIR/bin/pip install RPLCD==1.3.0 smbus2==0.4.3

echo ">>> [4/6] Creating optimized Python script..."
cat << EOF > $PYTHON_SCRIPT
import time
import socket
import fcntl
import struct
import sys
from RPLCD.i2c import CharLCD

# --- CONFIGURATION ---
I2C_ADDRESS = $I2C_ADDR
CHECK_INTERVAL = 60  # Check IP every 60 seconds (CPU saving)

def get_ip_address(ifname):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode('utf-8'))
        )[20:24])
    except (OSError, IOError):
        return None
    except Exception:
        return None

def main():
    try:
        # Initialize LCD
        lcd = CharLCD(i2c_expander='PCF8574', address=I2C_ADDRESS, port=1,
                      cols=16, rows=2, dotsize=8,
                      charmap='A00',
                      auto_linebreaks=True,
                      backlight_enabled=True)
    except (OSError, IOError) as e:
        print(f"LCD Initialization Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected Error during init: {e}")
        sys.exit(1)

    try:
        lcd.clear()
        lcd.write_string('Booting...')
    except (OSError, IOError):
        pass

    last_ip = ""

    while True:
        # Priority: WiFi (wlan0), then cable (eth0)
        current_ip = get_ip_address('wlan0')
        if not current_ip:
             current_ip = get_ip_address('eth0')
        
        if not current_ip:
            current_ip = "No network..."

        # Draw to screen ONLY if IP changed (saves I2C bus and CPU)
        if current_ip != last_ip:
            try:
                lcd.clear()
                lcd.write_string('MQTT Broker IP:')
                lcd.cursor_pos = (1, 0)
                lcd.write_string(current_ip)
                last_ip = current_ip
            except (OSError, IOError) as e:
                print(f"I2C Error: {e}")

        # RPi Zero 2W can sleep. This does not block the system.
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
EOF

echo ">>> [5/6] Creating Systemd service file..."
# Creating .service file dynamically injecting paths
# Added SupplementaryGroups=i2c to grant permissions immediately without re-login
sudo bash -c "cat << EOF > /etc/systemd/system/${SERVICE_NAME}.service
[Unit]
Description=LCD Display Service for SmartHome Broker
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
SupplementaryGroups=i2c
WorkingDirectory=$SCRIPT_DIR
# Run python from inside venv
ExecStart=$VENV_DIR/bin/python $PYTHON_SCRIPT
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF"

echo ">>> [6/6] Starting service..."
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
sudo systemctl restart $SERVICE_NAME

echo "-------------------------------------------------------"
echo "SUCCESS! Service '$SERVICE_NAME' has been installed."
echo "Check status with: systemctl status $SERVICE_NAME"
echo "Check logs with: journalctl -u $SERVICE_NAME -f"
echo "-------------------------------------------------------"
