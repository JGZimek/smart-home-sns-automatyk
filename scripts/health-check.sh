#!/usr/bin/env bash
# ==========================================================
# Smart Home – healthcheck / watchdog brokera (uruchamiany z timera)
# ==========================================================
# Co kilka minut sprawdza, czy broker MQTT nasluchuje. Jesli nie –
# restartuje kontener (samonaprawa). Lekki, bez zaleznosci od Pythona.
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
. "$SELF_DIR/lib/common.sh"
load_config

PORT="${MQTT_PORT:-1883}"
STATE="$STATE_DIR/broker_fail_count"
mkdir -p "$STATE_DIR"

fails=0
[ -f "$STATE" ] && fails="$(cat "$STATE" 2>/dev/null || echo 0)"

if nc -z localhost "$PORT" 2>/dev/null; then
  echo 0 > "$STATE"
  exit 0
fi

fails=$((fails + 1))
echo "$fails" > "$STATE"
warn "Broker nie odpowiada na porcie $PORT (nieudanych prob: $fails)."

# Po 2 kolejnych nieudanych probach – restart kontenera.
if [ "$fails" -ge 2 ]; then
  warn "Restartuje kontener brokera (samonaprawa)..."
  ( cd "$REPO_DIR" && compose restart mosquitto ) || ( cd "$REPO_DIR" && compose up -d )
  echo 0 > "$STATE"
fi
