#!/usr/bin/env bash
# ==========================================================
# Smart Home RPi – glowny instalator (idempotentny)
# ==========================================================
# Stawia kompletny serwer Smart Home na Ubuntu Server (Raspberry Pi Zero 2W):
# broker MQTT + mDNS + tuning + fallbacki + diagnostyka ESP + Tailscale.
#
# Mozna uruchamiac wielokrotnie (uzywany tez przez `smarthome update`).
# Kroki to ponumerowane moduly w setup.d/ – wykonywane po kolei.
#
#   sudo ./setup.sh                 # pelna instalacja
#   sudo ./setup.sh 40-broker       # tylko wybrany krok (po nazwie/prefiksie)
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
. "$SELF_DIR/lib/common.sh"

require_root

# Pierwsza instalacja: utworz katalog konfiguracji i skopiuj wzorzec env.
mkdir -p "$CONFIG_DIR" "$STATE_DIR"
if [ ! -f "$CONFIG_FILE" ]; then
  install -m 0640 "$SCRIPTS_DIR/smarthome.env.example" "$CONFIG_FILE"
  ok "Utworzono $CONFIG_FILE (z wartosci domyslnych) – mozesz go pozniej dostroic."
fi
load_config

step "Smart Home – instalacja serwera RPi (hostname: ${HOSTNAME:-rpi-smarthome})"
log "Repo: $REPO_DIR"

# Pozwol uruchomic pojedynczy krok: ./setup.sh 40   lub   ./setup.sh broker
FILTER="${1:-}"

shopt -s nullglob
ran=0
for mod in "$SCRIPTS_DIR"/setup.d/*.sh; do
  name="$(basename "$mod")"
  if [ -n "$FILTER" ] && [[ "$name" != *"$FILTER"* ]]; then
    continue
  fi
  step "Krok: $name"
  # shellcheck source=/dev/null
  . "$mod"
  ran=$((ran+1))
done

if [ "$ran" -eq 0 ]; then
  die "Nie znaleziono modulu pasujacego do '$FILTER' w setup.d/"
fi

step "Gotowe!"
ok "Serwer Smart Home zainstalowany. Szybka diagnostyka:  smarthome status"
log "Podglad ruchu MQTT:                                  smarthome watch"
log "Dashboard wezlow (gdy WEB_ENABLE=1):                 http://${HOSTNAME:-rpi-smarthome}.local:${WEB_PORT:-8080}"
