#!/usr/bin/env bash
# ==========================================================
# Smart Home RPi – wspolna biblioteka skryptow setupu
# ==========================================================
# Logowanie (PL), wykrywanie srodowiska, helpery idempotentne,
# ladowanie konfiguracji /etc/smarthome/smarthome.env.
#
# Uzycie: source "$(dirname "$0")/lib/common.sh"
# (sciezki ponizej dziala niezaleznie od katalogu wywolania)

# --- Sciezki kanoniczne (jedno zrodlo prawdy dla calego setupu) ---
# REPO_DIR  = katalog repozytorium (np. /opt/smarthome)
# SCRIPTS_DIR = .../scripts
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"
export SCRIPTS_DIR REPO_DIR

# Sciezki instalacyjne na docelowym systemie
export CONFIG_DIR="/etc/smarthome"
export CONFIG_FILE="$CONFIG_DIR/smarthome.env"
export STATE_DIR="/var/lib/smarthome"
export VENV_DIR="$SCRIPTS_DIR/.venv"
export SERVICE_USER="smarthome"

# --- Kolory (tylko gdy terminal) ---
if [ -t 1 ]; then
  C_RESET=$'\033[0m'; C_RED=$'\033[0;31m'; C_GREEN=$'\033[0;32m'
  C_YELLOW=$'\033[0;33m'; C_BLUE=$'\033[0;34m'; C_BOLD=$'\033[1m'
else
  C_RESET=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_BLUE=""; C_BOLD=""
fi

log()   { printf '%s[ smarthome ]%s %s\n' "$C_BLUE" "$C_RESET" "$*"; }
ok()    { printf '%s[    OK     ]%s %s\n' "$C_GREEN" "$C_RESET" "$*"; }
warn()  { printf '%s[   UWAGA   ]%s %s\n' "$C_YELLOW" "$C_RESET" "$*" >&2; }
err()   { printf '%s[   BLAD    ]%s %s\n' "$C_RED" "$C_RESET" "$*" >&2; }
step()  { printf '\n%s>>> %s%s\n' "$C_BOLD" "$*" "$C_RESET"; }
die()   { err "$*"; exit 1; }

# --- Wymagania srodowiska ---
require_root() {
  if [ "$(id -u)" -ne 0 ]; then
    die "Ten skrypt wymaga uprawnien root. Uruchom przez: sudo $0"
  fi
}

# Wczytaj konfiguracje uzytkownika (jesli istnieje) na wierzch wartosci domyslnych.
# Najpierw .example (defaulty), potem realny plik (nadpisuje).
load_config() {
  set -a
  # shellcheck disable=SC1090
  [ -f "$SCRIPTS_DIR/smarthome.env.example" ] && . "$SCRIPTS_DIR/smarthome.env.example"
  # shellcheck disable=SC1090
  [ -f "$CONFIG_FILE" ] && . "$CONFIG_FILE"
  set +a
}

# Sprawdza, czy polecenie istnieje
have() { command -v "$1" >/dev/null 2>&1; }

# Wybór polecenia compose: "docker compose" (plugin v2) lub "docker-compose" (legacy)
compose() {
  if docker compose version >/dev/null 2>&1; then
    docker compose "$@"
  elif have docker-compose; then
    docker-compose "$@"
  else
    die "Brak Docker Compose (ani plugin v2, ani docker-compose)."
  fi
}

# Idempotentny zapis pliku tylko gdy zawartosc sie rozni (mniej zapisow na karte SD).
# Uzycie: write_if_changed /sciezka/plik <<'EOF' ... EOF
write_if_changed() {
  local dest="$1" tmp
  tmp="$(mktemp)"
  cat > "$tmp"
  if [ -f "$dest" ] && cmp -s "$tmp" "$dest"; then
    rm -f "$tmp"
    return 1   # bez zmian
  fi
  install -D -m "${2:-0644}" "$tmp" "$dest"
  rm -f "$tmp"
  return 0     # zmieniono
}

# Czy uruchomione na Raspberry Pi / arm (informacyjnie, nie blokuje)
is_raspberry_pi() {
  grep -qi 'raspberry' /proc/cpuinfo /sys/firmware/devicetree/base/model 2>/dev/null
}
