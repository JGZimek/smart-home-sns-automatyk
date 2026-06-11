#!/usr/bin/env bash
# Krok 60: Tailscale – zdalny dostep SSH bez przekierowania portow / publicznego IP.

if [ "${TAILSCALE_ENABLE:-1}" != "1" ]; then
  log "Tailscale wylaczony w konfiguracji (TAILSCALE_ENABLE=0) – pomijam."
  return 0 2>/dev/null || exit 0
fi

if ! have tailscale; then
  log "Instaluje Tailscale (install.sh)..."
  curl -fsSL https://tailscale.com/install.sh | sh
  ok "Tailscale zainstalowany."
else
  ok "Tailscale juz zainstalowany."
fi
systemctl enable --now tailscaled >/dev/null 2>&1 || true

# Zbuduj argumenty 'tailscale up'
up_args=(--hostname "${HOSTNAME:-rpi-smarthome}")
[ "${TAILSCALE_SSH:-1}" = "1" ] && up_args+=(--ssh)

# Czy juz zalogowany?
if tailscale status >/dev/null 2>&1 && [ "$(tailscale status --json 2>/dev/null | jq -r '.BackendState' 2>/dev/null)" = "Running" ]; then
  ok "Tailscale juz polaczony: $(tailscale ip -4 2>/dev/null | head -n1)"
  # Mimo to dociagnij ustawienia (np. wlaczenie SSH) – idempotentnie.
  tailscale up "${up_args[@]}" >/dev/null 2>&1 || true
elif [ -n "${TAILSCALE_AUTHKEY:-}" ]; then
  log "Loguje Tailscale bezinteraktywnie (auth key z konfiguracji)..."
  if tailscale up --authkey "$TAILSCALE_AUTHKEY" "${up_args[@]}"; then
    ok "Tailscale polaczony: $(tailscale ip -4 2>/dev/null | head -n1)"
  else
    warn "Logowanie Tailscale auth-key nie powiodlo sie – sprawdz klucz."
  fi
else
  warn "Tailscale wymaga jednorazowego zalogowania. Uruchom recznie:"
  printf '       sudo tailscale up %s\n' "${up_args[*]}"
  log "Albo wklej pre-auth key do TAILSCALE_AUTHKEY w $CONFIG_FILE i puść 'smarthome update'."
fi
