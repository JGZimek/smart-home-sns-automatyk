#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== DIAGNOSTYKA SYSTEMU SMART HOME ===${NC}"

# Sprawdzenie Docker (Mosquitto)
if [ "$(sudo docker ps -q -f name=mqtt_broker)" ]; then
    echo -e "MQTT Broker (Docker): ${GREEN}Działa${NC}"
else
    echo -e "MQTT Broker (Docker): ${RED}NIE DZIAŁA${NC}"
fi

# Sprawdzenie usług systemd
services=("broker_lcd" "wifi_manager" "wifi-power-off")
for s in "${services[@]}"; do
    if systemctl is-active --quiet $s; then
        echo -e "Usługa $s: ${GREEN}OK${NC}"
    else
        echo -e "Usługa $s: ${RED}BŁĄD${NC}"
    fi
done

# Parametry sprzętowe
echo "Temperatura: $(vcgencmd measure_temp | cut -d '=' -f 2)"
echo "Swap: $(free -h | grep Swap | awk '{print $2}')"
echo "IP Lokalne: $(hostname -I)"