#!/usr/bin/env bash
# Krok 50: uslugi aplikacyjne (monitor wezlow + web, AP fallback, LCD, health timer) + CLI.

# --- 1. Uzytkownik systemowy 'smarthome' ---
if ! id "$SERVICE_USER" >/dev/null 2>&1; then
  log "Tworze uzytkownika systemowego '$SERVICE_USER'..."
  useradd --system --create-home --home-dir "$STATE_DIR" --shell /usr/sbin/nologin "$SERVICE_USER"
else
  ok "Uzytkownik '$SERVICE_USER' juz istnieje."
fi
# Grupy: docker (sterowanie brokerem), i2c/gpio (LCD). Brak grupy = pomin po cichu.
for grp in docker i2c gpio; do
  getent group "$grp" >/dev/null 2>&1 && usermod -aG "$grp" "$SERVICE_USER" || true
done
# Stan (nodes.json) musi byc zapisywalny przez usluge.
mkdir -p "$STATE_DIR"
chown -R "$SERVICE_USER:$SERVICE_USER" "$STATE_DIR"
# Katalog firmware dla OTA (serwowany po HTTP). setgid -> pliki dziedzicza grupe,
# zeby operator z grupy 'smarthome' mogl tu wgrywac binarki bez sudo.
mkdir -p "$STATE_DIR/firmware"
chown "$SERVICE_USER:$SERVICE_USER" "$STATE_DIR/firmware"
chmod 2775 "$STATE_DIR/firmware"

# --- 2. Virtualenv z zaleznosciami Pythona ---
if [ ! -x "$VENV_DIR/bin/python" ]; then
  log "Tworze virtualenv ($VENV_DIR)..."
  python3 -m venv "$VENV_DIR"
fi
log "Instaluje zaleznosci Pythona (paho-mqtt, Flask, opcjonalnie LCD)..."
"$VENV_DIR/bin/pip" install --quiet --upgrade pip
if ! "$VENV_DIR/bin/pip" install --quiet -r "$SCRIPTS_DIR/requirements.txt"; then
  warn "Pelna instalacja requirements nie powiodla sie (LCD?) – probuje bez zaleznosci LCD."
  "$VENV_DIR/bin/pip" install --quiet paho-mqtt==1.6.1 Flask==3.0.3
fi
ok "Srodowisko Pythona gotowe."

# --- 3. Instalacja unitow systemd ---
log "Instaluje uslugi systemd..."
install -m 0644 "$SCRIPTS_DIR"/systemd/*.service "$SCRIPTS_DIR"/systemd/*.timer /etc/systemd/system/
systemctl daemon-reload

# Monitor wezlow + web status (zawsze)
systemctl enable --now smarthome-node-monitor.service
ok "Usluga monitora wezlow aktywna."

# Health timer (watchdog brokera + publikacja zdrowia Pi)
systemctl enable --now smarthome-health.timer
ok "Timer health (watchdog brokera) aktywny."

# AP fallback – tylko gdy SWIADOMIE wlaczony (wymaga NetworkManager na wlan0).
if [ "${AP_FALLBACK_ENABLE:-0}" = "1" ]; then
  if ! have nmcli; then
    warn "AP fallback wymaga NetworkManager – instaluje pakiet."
    warn "Jesli laczysz sie przez Wi-Fi pod networkd, NAJPIERW zmigruj sie do NM (Etap 3.5)!"
    wait_for_apt
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq network-manager || \
      warn "Instalacja network-manager nie powiodla sie."
  fi
  systemctl enable --now smarthome-wifi-fallback.service
  ok "Awaryjny Access Point Wi-Fi aktywny (SSID: ${AP_SSID:-SmartHome-Config})."
else
  systemctl disable --now smarthome-wifi-fallback.service >/dev/null 2>&1 || true
  log "AP fallback wylaczony (AP_FALLBACK_ENABLE=0) – domyslnie. Wlacz po migracji Wi-Fi do NM."
fi

# LCD – auto-detekcja / wymuszenie / wylaczenie
lcd_wanted=0
case "${LCD_ENABLE:-auto}" in
  1) lcd_wanted=1 ;;
  0) lcd_wanted=0 ;;
  auto|*)
    # Wykryj urzadzenie I2C pod skonfigurowanym adresem.
    addr_short="${LCD_I2C_ADDR:-0x27}"; addr_short="${addr_short#0x}"
    if have i2cdetect && i2cdetect -y 1 2>/dev/null | grep -qiE "\b${addr_short}\b"; then
      lcd_wanted=1
    fi
    ;;
esac
if [ "$lcd_wanted" = "1" ]; then
  systemctl enable --now smarthome-lcd.service
  ok "Usluga LCD aktywna (adres ${LCD_I2C_ADDR:-0x27})."
else
  systemctl disable --now smarthome-lcd.service >/dev/null 2>&1 || true
  log "LCD pominiety (nie wykryto / wylaczony)."
fi

# --- 4. CLI 'smarthome' dostepne globalnie ---
ln -sf "$SCRIPTS_DIR/bin/smarthome" /usr/local/bin/smarthome
chmod +x "$SCRIPTS_DIR/bin/smarthome" 2>/dev/null || true
ok "CLI dostepne: 'smarthome status'"
