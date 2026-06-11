#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Home – opcjonalny wyswietlacz LCD I2C (16x2).

Pokazuje rotacyjnie: IP/hostname, stan brokera + liczbe wezlow online, IP Tailscale,
ew. tryb awaryjnego AP. Czyta stan z /var/lib/smarthome/nodes.json (zapisywany przez
node_monitor). Jesli brak biblioteki LCD albo urzadzenia I2C – konczy sie czysto
(usluga i tak jest wlaczana tylko po wykryciu LCD w setupie).

Konfiguracja (env): LCD_I2C_ADDR, STATE_FILE, HOSTNAME.
"""
import json
import os
import socket
import subprocess
import time

ADDR = int(os.environ.get("LCD_I2C_ADDR", "0x27"), 16)
STATE_FILE = os.environ.get("STATE_FILE", "/var/lib/smarthome/nodes.json")
HOSTNAME = os.environ.get("HOSTNAME", socket.gethostname())
ROTATE_S = 4


def log(msg):
    print("[lcd] %s" % msg, flush=True)


try:
    from RPLCD.i2c import CharLCD
except Exception as e:  # brak biblioteki – nie blokuj systemu
    log("Brak biblioteki RPLCD (%s) – usluga LCD konczy sie." % e)
    raise SystemExit(0)


def primary_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


def tailscale_ip():
    try:
        out = subprocess.run(["tailscale", "ip", "-4"], capture_output=True,
                             text=True, timeout=3)
        lines = out.stdout.strip().splitlines()
        return lines[0] if lines else None
    except Exception:
        return None


def read_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def ap_active():
    try:
        out = subprocess.run(["nmcli", "-t", "-f", "NAME", "connection", "show",
                              "--active"], capture_output=True, text=True, timeout=3)
        return "SmartHome-Config" in out.stdout
    except Exception:
        return False


def screens():
    """Zwraca liste ekranow (2 linie kazdy) do rotacji."""
    out = []
    if ap_active():
        out.append(("TRYB AWARYJNY", "AP:SmartHome-Cfg"))
        return out

    ip = primary_ip()
    out.append((HOSTNAME[:16], ("IP:%s" % ip) if ip else "Brak sieci"))

    st = read_state()
    if st:
        nodes = st.get("nodes", {})
        online = sum(1 for n in nodes.values() if n.get("status") == "online")
        total = len(nodes)
        out.append(("Broker: OK", "Wezly %d/%d ON" % (online, total)))
    else:
        out.append(("Broker: ?", "Brak danych"))

    ts = tailscale_ip()
    if ts:
        out.append(("Tailscale", ts[:16]))
    return out


def main():
    try:
        lcd = CharLCD(i2c_expander="PCF8574", address=ADDR, port=1, cols=16, rows=2,
                      charmap="A00", auto_linebreaks=False, backlight_enabled=True)
    except Exception as e:
        log("Nie udalo sie zainicjowac LCD pod adresem 0x%02x: %s" % (ADDR, e))
        raise SystemExit(0)

    lcd.clear()
    lcd.write_string("Smart Home OS")
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Start...")
    time.sleep(2)

    last = None
    while True:
        for line1, line2 in screens():
            cur = (line1, line2)
            if cur != last:
                try:
                    lcd.clear()
                    lcd.write_string(line1[:16])
                    lcd.cursor_pos = (1, 0)
                    lcd.write_string(line2[:16])
                    last = cur
                except Exception:
                    pass
            time.sleep(ROTATE_S)


if __name__ == "__main__":
    main()
