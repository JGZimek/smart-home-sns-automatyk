#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Home – awaryjny Access Point Wi-Fi (NetworkManager / Ubuntu).

Gdy Pi NIE MA aktywnego polaczenia z siecia (brak Ethernetu i brak polaczenia
Wi-Fi jako klient) przez okres karencji, podnosi wlasny Access Point i serwuje
prosta strone do wpisania nowej sieci Wi-Fi. Po udanym polaczeniu kasuje AP.

WAZNE: wyzwalaczem jest brak polaczenia LAN, a NIE brak internetu. Makieta na
targach moze dzialac calkowicie offline (Pi jako jedyny wezel sieci) – w takim
ukladzie AP fallback nie ma sie odpalac falszywie. Liczy sie, czy Pi jest do
czegokolwiek podlaczone w warstwie L2/L3.

Konfiguracja (env z /etc/smarthome/smarthome.env):
  AP_SSID, AP_PASS (puste = open), AP_GRACE_S
Wymaga uprawnien root (nmcli). Uruchamiana jako usluga smarthome-wifi-fallback.
"""
import os
import signal
import subprocess
import threading
import time

from flask import Flask, request

AP_SSID = os.environ.get("AP_SSID", "SmartHome-Config")
AP_PASS = os.environ.get("AP_PASS", "").strip()
GRACE_S = int(os.environ.get("AP_GRACE_S", "90"))
CHECK_INTERVAL_S = 15
PORT = 80

app = Flask(__name__)
_ap_active = threading.Event()


def log(msg):
    print("[wifi_fallback] %s" % msg, flush=True)


def nmcli(*args, check=False):
    return subprocess.run(["nmcli", *args], capture_output=True, text=True, check=check)


def nm_manages_wifi():
    """True tylko gdy NetworkManager zarzadza jakims interfejsem Wi-Fi.
    Na Ubuntu Server (networkd) wlan0 bywa 'unmanaged' – wtedy NIE ruszamy sieci."""
    try:
        out = nmcli("-t", "-f", "DEVICE,TYPE,STATE", "device")
    except FileNotFoundError:
        return False
    for line in out.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) >= 3 and parts[1] == "wifi" and parts[2] != "unmanaged":
            return True
    return False


def has_connectivity():
    """True, jesli Pi ma aktywne polaczenie sieciowe (eth lub wifi-klient)."""
    # 1. Aktywne urzadzenia inne niz nasz AP / loopback.
    out = nmcli("-t", "-f", "DEVICE,TYPE,STATE", "device")
    for line in out.stdout.strip().splitlines():
        parts = line.split(":")
        if len(parts) < 3:
            continue
        dev, dtype, state = parts[0], parts[1], parts[2]
        if dtype in ("ethernet", "wifi") and state == "connected":
            # Upewnij sie, ze to nie nasz wlasny AP.
            act = nmcli("-t", "-f", "GENERAL.CONNECTION", "device", "show", dev)
            if AP_SSID in act.stdout:
                continue
            return True
    return False


def start_ap():
    if _ap_active.is_set():
        return True
    log("Brak polaczenia – podnosze Access Point '%s'" % AP_SSID)
    nmcli("connection", "delete", AP_SSID)  # wyczysc ewentualne resztki
    r = nmcli("connection", "add", "type", "wifi", "ifname", "wlan0",
              "con-name", AP_SSID, "autoconnect", "no", "ssid", AP_SSID,
              "mode", "ap")
    if r.returncode != 0:
        log("Nie udalo sie utworzyc AP: %s" % r.stderr.strip())
        return False
    nmcli("connection", "modify", AP_SSID, "802-11-wireless.band", "bg",
          "ipv4.method", "shared")
    if len(AP_PASS) >= 8:
        nmcli("connection", "modify", AP_SSID, "wifi-sec.key-mgmt", "wpa-psk",
              "wifi-sec.psk", AP_PASS)
    else:
        nmcli("connection", "modify", AP_SSID, "wifi-sec.key-mgmt", "none")
        if AP_PASS:
            log("AP_PASS krotsze niz 8 znakow – uruchamiam AP jako otwarty.")
    r = nmcli("connection", "up", AP_SSID)
    if r.returncode != 0:
        log("Nie udalo sie wlaczyc AP: %s" % r.stderr.strip())
        return False
    _ap_active.set()
    log("AP aktywny. Polacz sie z '%s' i otworz http://10.42.0.1" % AP_SSID)
    return True


def stop_ap():
    if not _ap_active.is_set():
        return
    log("Wykryto polaczenie – wylaczam AP '%s'" % AP_SSID)
    nmcli("connection", "down", AP_SSID)
    nmcli("connection", "delete", AP_SSID)
    _ap_active.clear()


def connect_wifi(ssid, password):
    log("Probuje polaczyc z siecia '%s'..." % ssid)
    stop_ap()
    time.sleep(2)
    args = ["device", "wifi", "connect", ssid]
    if password:
        args += ["password", password]
    r = nmcli(*args)
    if r.returncode == 0:
        log("Polaczono z '%s'." % ssid)
        return True
    log("Polaczenie nieudane: %s" % r.stderr.strip())
    return False


PAGE = """<!DOCTYPE html><html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Home – konfiguracja Wi-Fi</title>
<style>body{font-family:system-ui,sans-serif;background:#0f1620;color:#e6edf3;text-align:center;padding:24px}
.box{background:#161e29;max-width:380px;margin:0 auto;padding:24px;border-radius:12px;border:1px solid #2a3441}
h2{margin-top:0}input{width:100%%;padding:12px;margin:8px 0;border-radius:6px;border:1px solid #2a3441;background:#0f1620;color:#e6edf3;box-sizing:border-box}
button{width:100%%;padding:12px;background:#238636;color:#fff;border:0;border-radius:6px;font-size:16px;cursor:pointer}
p{color:#8b98a5;font-size:14px}</style></head><body>
<div class="box"><h2>Smart Home – Wi-Fi</h2>
<p>Serwer nie ma polaczenia z siecia. Podaj dane Wi-Fi (pasmo 2.4&nbsp;GHz).</p>
<form action="/connect" method="post">
<input name="ssid" placeholder="Nazwa sieci (SSID)" required>
<input name="password" type="password" placeholder="Haslo Wi-Fi">
<button type="submit">Polacz</button></form></div></body></html>"""


@app.route("/")
def index():
    return PAGE


# Przechwyc typowe sondy captive-portal, by telefon sam otworzyl strone.
@app.route("/generate_204")
@app.route("/hotspot-detect.html")
@app.route("/ncsi.txt")
def captive():
    return PAGE


@app.route("/connect", methods=["POST"])
def connect():
    ssid = request.form.get("ssid", "").strip()
    password = request.form.get("password", "")

    def worker():
        time.sleep(2)
        if not connect_wifi(ssid, password):
            start_ap()  # nieudane – wroc do trybu AP
        os._exit(0)  # zrestartuj proces (systemd Restart=always) z czystym stanem

    threading.Thread(target=worker, daemon=True).start()
    return ("<body style='font-family:system-ui;text-align:center;padding:48px;background:#0f1620;color:#e6edf3'>"
            "<h2>Zapisuje...</h2><p>Probuje polaczyc z <b>%s</b>. Ta siec (%s) zaraz zniknie.</p></body>"
            % (ssid, AP_SSID))


def web_server():
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    except Exception as e:
        log("Serwer www padl: %s" % e)


def main():
    log("Start. AP fallback SSID='%s', karencja=%ss" % (AP_SSID, GRACE_S))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))

    # Zabezpieczenie: jesli NM nie zarzadza Wi-Fi (np. Ubuntu Server pod networkd),
    # NIE ruszamy interfejsu – tylko czekamy, az ktos zmigruje sie do NM.
    warned = False
    while not nm_manages_wifi():
        if not warned:
            log("NetworkManager nie zarzadza Wi-Fi (interfejs 'unmanaged' lub brak nmcli).")
            log("AP fallback BEZCZYNNY. Zmigruj Wi-Fi do NM (Etap 3.5), aby go uzywac.")
            warned = True
        time.sleep(60)

    threading.Thread(target=web_server, daemon=True).start()

    down_since = None
    while True:
        if not nm_manages_wifi():
            time.sleep(CHECK_INTERVAL_S)
            continue
        if has_connectivity():
            down_since = None
            if _ap_active.is_set():
                stop_ap()
        else:
            if down_since is None:
                down_since = time.time()
                log("Utracono polaczenie – start odliczania karencji (%ss)." % GRACE_S)
            elif time.time() - down_since >= GRACE_S:
                start_ap()
        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
