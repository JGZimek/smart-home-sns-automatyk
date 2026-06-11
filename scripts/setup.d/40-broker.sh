#!/usr/bin/env bash
# Krok 40: broker MQTT (Mosquitto w Dockerze) + konto dla ESP-ek.

MOSQ_CFG="$REPO_DIR/mosquitto/config"
PASSWD_FILE="$MOSQ_CFG/passwd"
B_USER="${BROKER_USER:-esp32}"
B_PASS="${BROKER_PASS:-esp32}"

mkdir -p "$MOSQ_CFG" "$REPO_DIR/mosquitto/data" "$REPO_DIR/mosquitto/log"

# --- Konto brokera (broker ma allow_anonymous false; firmware loguje sie B_USER/B_PASS) ---
# Generujemy haslo komenda mosquitto_passwd uruchomiona w kontenerze – bez instalacji
# mosquitto na hoscie. Plik passwd jest poza gitem (gitignore).
if [ ! -f "$PASSWD_FILE" ] || ! grep -q "^${B_USER}:" "$PASSWD_FILE" 2>/dev/null; then
  log "Generuje konto brokera dla uzytkownika '$B_USER'..."
  docker run --rm -v "$MOSQ_CFG:/cfg" eclipse-mosquitto:2 \
    mosquitto_passwd -b -c /cfg/passwd "$B_USER" "$B_PASS"
  chmod 0640 "$PASSWD_FILE" 2>/dev/null || true
  ok "Konto brokera utworzone ($PASSWD_FILE)."
else
  ok "Konto brokera '$B_USER' juz istnieje."
fi

# --- Start / aktualizacja kontenera brokera ---
log "Uruchamiam brokera MQTT (docker compose up -d)..."
( cd "$REPO_DIR" && compose up -d )

# Krotka weryfikacja, ze port nasluchuje
sleep 2
if nc -z localhost "${MQTT_PORT:-1883}" 2>/dev/null; then
  ok "Broker MQTT nasluchuje na porcie ${MQTT_PORT:-1883}."
else
  warn "Broker jeszcze nie odpowiada na ${MQTT_PORT:-1883} – sprawdz 'smarthome broker logs'."
fi
