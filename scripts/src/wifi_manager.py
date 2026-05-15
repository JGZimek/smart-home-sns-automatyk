import time
import subprocess
import os
import signal
import threading
from flask import Flask, request, render_template_string

CHECK_INTERVAL = 60
AP_SSID = "SmartHome-Config"
PORT = 80

app = Flask(__name__)

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
    try:
        subprocess.check_call(['ping', '-c', '1', '-W', '1', '8.8.8.8'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except:
        return False

def start_open_ap():
    print(f">>> Starting Open AP: {AP_SSID}")
    try:
        subprocess.run(['nmcli', 'con', 'delete', AP_SSID], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['nmcli', 'con', 'add', 'type', 'wifi', 'ifname', 'wlan0', 'con-name', AP_SSID, 'ssid', AP_SSID, 'mode', 'ap'], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['nmcli', 'con', 'modify', AP_SSID, 'wifi-sec.key-mgmt', 'none'], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['nmcli', 'con', 'modify', AP_SSID, 'ipv4.method', 'shared'], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['nmcli', 'con', 'up', AP_SSID], check=True, stdout=subprocess.DEVNULL)
        return True
    except Exception as e:
        print(f"Error creating AP: {e}")
        return False

def connect_to_wifi(ssid, password):
    print(f">>> Attempting connection to: {ssid}")
    try:
        subprocess.run(['nmcli', 'con', 'delete', AP_SSID], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password], check=True)
        return True
    except:
        return False

@app.route('/')
def home():
    return render_template_string(HTML_PAGE)

@app.route('/connect', methods=['POST'])
def handle_connect():
    ssid = request.form.get('ssid')
    password = request.form.get('password')
    
    def switch_network():
        time.sleep(3)
        connect_to_wifi(ssid, password)
        os.kill(os.getpid(), signal.SIGTERM)

    threading.Thread(target=switch_network).start()
    
    return f"<div style='font-family:sans-serif;text-align:center;padding:50px;'><h1>Saving...</h1><p>Attempting to connect to <b>{ssid}</b>.</p></div>"

def main():
    time.sleep(5)
    while True:
        if check_internet():
            time.sleep(CHECK_INTERVAL)
        else:
            if start_open_ap():
                try:
                    app.run(host='0.0.0.0', port=PORT)
                except:
                    time.sleep(10)
            time.sleep(10)

if __name__ == "__main__":
    main()