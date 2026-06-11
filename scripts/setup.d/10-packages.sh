#!/usr/bin/env bash
# Krok 10: pakiety systemowe + Docker.
# Sourcowany przez setup.sh (ma dostep do helperow z lib/common.sh).

export DEBIAN_FRONTEND=noninteractive

log "Aktualizacja listy pakietow (apt update)..."
apt-get update -qq

# Pakiety z repo Ubuntu. NIE instalujemy mosquitto-server (broker chodzi w Dockerze),
# ale mosquitto-clients sa potrzebne dla CLI 'smarthome' i healthcheckow.
PKGS=(
  avahi-daemon          # mDNS – ESP-ki znajduja brokera jako rpi-smarthome.local
  network-manager       # nmcli – tuning Wi-Fi i awaryjny AP
  mosquitto-clients     # mosquitto_pub/sub dla CLI i diagnostyki
  zram-tools            # kompresowany swap w RAM (wlasciwe dla 512MB Zero 2W)
  i2c-tools             # wykrywanie LCD (i2cdetect)
  jq                    # parsowanie JSON w CLI
  python3-venv          # srodowisko dla uslug Pythona
  python3-dev           # build smbus2/RPLCD jesli potrzeba
  netcat-openbsd        # nc – healthcheck portu brokera
  curl ca-certificates
)
log "Instalacja pakietow: ${PKGS[*]}"
apt-get install -y -qq "${PKGS[@]}"
ok "Pakiety systemowe zainstalowane."

# --- Docker (oficjalny skrypt – ogarnia arm64 i plugin compose v2) ---
if have docker && docker compose version >/dev/null 2>&1; then
  ok "Docker + compose juz obecne ($(docker --version | awk '{print $3}' | tr -d ','))."
else
  log "Instalacja Dockera (get.docker.com)..."
  curl -fsSL https://get.docker.com | sh
  ok "Docker zainstalowany."
fi
systemctl enable --now docker >/dev/null 2>&1 || true
