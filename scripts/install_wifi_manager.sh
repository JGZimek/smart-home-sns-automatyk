#!/bin/bash

# --- CONFIGURATION ---
SCRIPT_DIR=$(pwd)
VENV_DIR="$SCRIPT_DIR/wifi_env"
PYTHON_SCRIPT="$SCRIPT_DIR/wifi_manager.py"
SERVICE_NAME="wifi_manager"
# Network should be open (no password), like ESP32 WiFiManager
HOTSPOT_SSID="SmartHome-Config"
# --------------------

echo ">>> [1/6] Updating system and installing dependencies..."
# Network-manager is required to control WiFi from console
sudo apt update
sudo apt install -y python3-pip python3-venv network-manager

echo ">>> [2/6] Creating virtual environment..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv $VENV_DIR
fi

echo ">>> [3/6] Installing Flask in venv..."
$VENV_DIR/bin/pip install Flask==3.0.3

echo ">>> [4/6] Generating WiFi Manager script (Python)..."
cat << EOF > $PYTHON_SCRIPT
import time
import subprocess
import os
import signal
import threading
from flask import Flask, request, render_template_string

# --- SETTINGS ---
CHECK_INTERVAL = 60       # Check internet every 60 seconds
AP_SSID = "$HOTSPOT_SSID" # Open Hotspot Name
PORT = 80                 # HTTP Port

app = Flask(__name__)

# Simple Responsive HTML Page
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>SmartHome Config</title>
    <style>
        body { font-family: sans-serif; padding: 20px; background: #eee; text-align: center; }
        .box { background: #fff; padding: 25px; border-radius: 10px; max-width: 400px; margin: 0 auto; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        h2 { margin-top: 0; color: #333; }
        input { width: 100%; padding: 12px; margin: 10px 0; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
        button { width: 100%; padding: 12px; background: #28a745; color: white; border: none; border-radius: 4px; font-size: 16px; cursor: pointer; }
        button:hover { background: #218838; }
    </style>
</head>
<body>
    <div class="box">
        <h2>WiFi Network Missing</h2>
        <p>RPi cannot access the internet. Please configure a new WiFi connection.</p>
        <form action="/connect" method="post">
            <input type="text" name="ssid" placeholder="Network Name (SSID)" required>
            <input type="password" name="password" placeholder="WiFi Password" required>
            <button type="submit">Connect</button>
        </form>
    </div>
</body>
</html>
"""

def check_internet():
    """Checks connection by pinging Google DNS (8.8.8.8)."""
    try:
        subprocess.check_call(['ping', '-c', '1', '-W', '1', '8.8.8.8'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except:
        return False

def start_open_ap():
    """Creates OPEN Access Point using NetworkManager."""
    print(f">>> Starting Open AP: {AP_SSID}")
    try:
        # 1. Remove old connection if exists
        subprocess.run(['nmcli', 'con', 'delete', AP_SSID], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # 2. Add new hotspot connection (no password - key-mgmt none)
        # Create profile
        subprocess.run([
            'nmcli', 'con', 'add', 'type', 'wifi', 'ifname', 'wlan0', 
            'con-name', AP_SSID, 'ssid', AP_SSID, 'mode', 'ap'
        ], check=True, stdout=subprocess.DEVNULL)

        # Set security to Open (None)
        subprocess.run(['nmcli', 'con', 'modify', AP_SSID, 'wifi-sec.key-mgmt', 'none'], check=True, stdout=subprocess.DEVNULL)
        
        # Set shared IP (to enable DHCP server for client)
        subprocess.run(['nmcli', 'con', 'modify', AP_SSID, 'ipv4.method', 'shared'], check=True, stdout=subprocess.DEVNULL)

        # 3. Bring up connection
        subprocess.run(['nmcli', 'con', 'up', AP_SSID], check=True, stdout=subprocess.DEVNULL)
        return True
    except Exception as e:
        print(f"Error creating AP: {e}")
        return False

def connect_to_wifi(ssid, password):
    """Closes AP and connects to provided network."""
    print(f">>> Attempting connection to: {ssid}")
    try:
        # Delete AP profile
        subprocess.run(['nmcli', 'con', 'delete', AP_SSID], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        # Connect to new network
        subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password], check=True)
        return True
    except:
        print("Failed to connect to WiFi (Wrong password or range issue).")
        return False

@app.route('/')
def home():
    return render_template_string(HTML_PAGE)

@app.route('/connect', methods=['POST'])
def handle_connect():
    ssid = request.form.get('ssid')
    password = request.form.get('password')
    
    # Run connection process in background to return HTML response first
    def switch_network():
        time.sleep(3) # Time to send HTML response
        success = connect_to_wifi(ssid, password)
        # Kill python process - systemd will restart it.
        # After restart, script checks for internet.
        # If connect_to_wifi succeeded -> Online -> loop waits.
        # If failed -> Offline -> AP starts again.
        os.kill(os.getpid(), signal.SIGTERM)

    threading.Thread(target=switch_network).start()
    
    return f"""
    <div style='font-family:sans-serif;text-align:center;padding:50px;'>
        <h1>Saving...</h1>
        <p>RPi is attempting to connect to <b>{ssid}</b>.</p>
        <p>Access Point will be disabled now.</p>
        <p>Please reconnect your phone to your home WiFi.</p>
    </div>
    """

def main():
    print("--- WiFi Manager Start ---")
    time.sleep(5) # Short delay for system startup

    while True:
        if check_internet():
            print(f"ONLINE. Checking again in {CHECK_INTERVAL}s...")
            time.sleep(CHECK_INTERVAL)
        else:
            print("OFFLINE! Starting Rescue Mode (AP)...")
            if start_open_ap():
                print(f"AP Ready: {AP_SSID} (Open)")
                print(f"Web Server on port {PORT}")
                
                # Start Flask (blocks thread until restart in handle_connect)
                # host 0.0.0.0 allows external connections
                try:
                    app.run(host='0.0.0.0', port=PORT)
                except Exception as e:
                    print(f"Web Server Error: {e}")
                    time.sleep(10) # Prevent error loop
            
            time.sleep(10)

if __name__ == "__main__":
    main()
EOF

echo ">>> [5/6] Creating Systemd service..."
# Must run as root to manage NetworkManager
sudo bash -c "cat << EOF > /etc/systemd/system/${SERVICE_NAME}.service
[Unit]
Description=SmartHome WiFi Fallback Manager (Open AP)
After=network.target NetworkManager.service
Wants=network.target

[Service]
Type=simple
User=root
WorkingDirectory=$SCRIPT_DIR
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
echo "SUCCESS!"
echo "If RPi loses internet, you will see WiFi: $HOTSPOT_SSID"
echo "Connect to it and go to: http://10.42.0.1"
echo "-------------------------------------------------------"
