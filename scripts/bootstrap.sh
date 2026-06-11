#!/usr/bin/env bash
# ==========================================================
# Smart Home RPi – bootstrap (one-liner)
# ==========================================================
# Jedyne polecenie potrzebne na swiezym Ubuntu Server:
#
#   curl -fsSL https://raw.githubusercontent.com/JGZimek/smart-home-sns-automatyk/develop/scripts/bootstrap.sh | sudo bash
#
# Co robi: instaluje git, klonuje (lub aktualizuje) repo do /opt/smarthome
# na wybranej galezi, po czym uruchamia scripts/setup.sh (wlasciwa instalacja).
#
# Zmienne srodowiskowe (opcjonalne):
#   REPO   – URL repo (domyslnie GitHub projektu)
#   BRANCH – galaz (domyslnie 'develop')
#   DEST   – katalog docelowy (domyslnie /opt/smarthome)
set -euo pipefail

REPO="${REPO:-https://github.com/JGZimek/smart-home-sns-automatyk.git}"
BRANCH="${BRANCH:-develop}"
DEST="${DEST:-/opt/smarthome}"

if [ "$(id -u)" -ne 0 ]; then
  echo "[BLAD] Uruchom przez sudo:  curl ... | sudo bash" >&2
  exit 1
fi

echo ">>> Smart Home bootstrap: galaz=$BRANCH cel=$DEST"

# git jest potrzebny do klonowania
if ! command -v git >/dev/null 2>&1; then
  echo ">>> Instaluje git..."
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq git
fi

if [ -d "$DEST/.git" ]; then
  echo ">>> Repo juz istnieje – aktualizuje ($DEST)..."
  git -C "$DEST" fetch --depth 1 origin "$BRANCH"
  git -C "$DEST" checkout "$BRANCH"
  git -C "$DEST" reset --hard "origin/$BRANCH"
else
  echo ">>> Klonuje repo do $DEST..."
  mkdir -p "$(dirname "$DEST")"
  git clone --depth 1 --branch "$BRANCH" "$REPO" "$DEST"
fi

echo ">>> Uruchamiam instalator (setup.sh)..."
exec bash "$DEST/scripts/setup.sh"
