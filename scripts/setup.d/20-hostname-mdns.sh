#!/usr/bin/env bash
# Krok 20: tozsamosc serwera (hostname) + mDNS (avahi).
# KLUCZOWE: firmware ESP32 szuka brokera po nazwie 'rpi-smarthome' (module_config.h).
# Bez tego moduly nie znajda serwera.

WANT_HOST="${HOSTNAME:-rpi-smarthome}"
CUR_HOST="$(hostnamectl --static 2>/dev/null || hostname)"

if [ "$CUR_HOST" != "$WANT_HOST" ]; then
  log "Ustawiam hostname: $CUR_HOST -> $WANT_HOST"
  hostnamectl set-hostname "$WANT_HOST"
else
  ok "Hostname juz ustawiony ($WANT_HOST)."
fi

# Wpis w /etc/hosts (zeby 'sudo' nie marudzilo i lokalne rozwiazywanie dzialalo).
if ! grep -qE "127\.0\.1\.1[[:space:]].*\b${WANT_HOST}\b" /etc/hosts 2>/dev/null; then
  log "Dodaje $WANT_HOST do /etc/hosts"
  if grep -qE '^127\.0\.1\.1' /etc/hosts; then
    sed -i -E "s/^(127\.0\.1\.1[[:space:]].*)$/\1 ${WANT_HOST}/" /etc/hosts
  else
    printf '127.0.1.1\t%s\n' "$WANT_HOST" >> /etc/hosts
  fi
fi

# Avahi = mDNS responder. Dzieki niemu ${WANT_HOST}.local jest rozglaszane w LAN.
log "Wlaczam avahi-daemon (mDNS)..."
systemctl enable --now avahi-daemon >/dev/null 2>&1
ok "mDNS aktywne – serwer widoczny jako ${WANT_HOST}.local"
