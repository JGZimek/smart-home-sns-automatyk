#!/bin/bash
set -e

echo ">>> Rozpoczynanie pełnej instalacji SmartHome..."

# 1. Optymalizacje hosta (Wyłączenie oszczędzania energii Wi-Fi jako pierwsze!)
echo ">>> Stosowanie optymalizacji systemu..."
chmod +x ./host_config.sh
sudo ./host_config.sh

# 2. Aktualizacja i zależności systemowe
echo ">>> Pobieranie pakietów systemowych..."
sudo apt update
sudo apt install -y docker.io docker-compose i2c-tools python3-pip python3-venv network-manager wireless-tools

# 3. Konfiguracja Docker (Mosquitto)
echo ">>> Uruchamianie brokera MQTT (Docker)..."
# Dodajemy obsługę błędu na wypadek, gdyby katalog domowy nie pozwalał na łatwy start compose w tym miejscu
cd /home/smarthome/smart-home-sns-automatyk
sudo docker-compose up -d
cd scripts/

# 4. Instalacja usług Python (LCD i WiFi Manager)
echo ">>> Konfiguracja środowisk Python..."
mkdir -p ./venv
python3 -m venv ./venv/smarthome_env
./venv/smarthome_env/bin/pip install RPLCD==1.3.0 smbus2==0.4.3 Flask==3.0.3

# 5. Kopiowanie i aktywacja usług systemowych
echo ">>> Konfiguracja systemd..."
sudo cp ./services/*.service /etc/systemd/system/
sudo systemctl daemon-reload

services=("broker_lcd" "wifi_manager" "wifi-power-off")
for s in "${services[@]}"; do
    sudo systemctl enable $s
    sudo systemctl restart $s
done

echo ">>> INSTALACJA ZAKOŃCZONA! System jest gotowy."