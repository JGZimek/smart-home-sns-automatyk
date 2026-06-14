#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart Home – monitor wezlow ESP + zdrowie Pi + web status.

Subskrybuje retained "wizytowki" i heartbeaty wszystkich wezlow (home/+/info,
home/+/availability, home/+/diag), buduje rejestr stanu, wykrywa wezly offline/stale
i brakujace, zapisuje go do /var/lib/smarthome/nodes.json (czyta CLI i LCD) oraz
serwuje lekki dashboard HTTP/JSON. Dodatkowo publikuje zdrowie samego Pi jako
"wezel" home/system/server/* – backend widzi serwer tak samo jak ESP-ki.

Konfiguracja przez zmienne srodowiskowe (z /etc/smarthome/smarthome.env):
  BROKER_USER, BROKER_PASS, MQTT_PORT, EXPECTED_KINDS, DIAG_STALE_S,
  WEB_ENABLE, WEB_PORT, HOSTNAME
"""
import json
import os
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

# ---------------- Konfiguracja ----------------
BROKER_HOST = os.environ.get("BROKER_HOST", "127.0.0.1")
BROKER_PORT = int(os.environ.get("MQTT_PORT", "1883"))
BROKER_USER = os.environ.get("BROKER_USER", "smarthome")
BROKER_PASS = os.environ.get("BROKER_PASS", "smarthome")
EXPECTED_KINDS = [k.strip() for k in os.environ.get(
    "EXPECTED_KINDS", "security,access,environment").split(",") if k.strip()]
DIAG_STALE_S = int(os.environ.get("DIAG_STALE_S", "90"))
WEB_ENABLE = os.environ.get("WEB_ENABLE", "1") == "1"
WEB_PORT = int(os.environ.get("WEB_PORT", "8080"))
# Przycisk "Aktualizuj serwer" w dashboardzie (POST /update -> plik-wyzwalacz ->
# rootowa usluga smarthome-update). Wylacz (0), jesli dashboard jest wystawiony
# szerzej niz zaufany LAN/Tailscale – to zdalne wywolanie aktualizacji jako root.
WEB_UPDATE_ENABLE = os.environ.get("WEB_UPDATE_ENABLE", "1") == "1"
HOSTNAME = os.environ.get("HOSTNAME", socket.gethostname())
STATE_FILE = os.environ.get("STATE_FILE", "/var/lib/smarthome/nodes.json")
# Plik-wyzwalacz samoaktualizacji (tworzy go web jako user 'smarthome', usuwa root).
UPDATE_REQUEST_FILE = os.environ.get("UPDATE_REQUEST_FILE", "/var/lib/smarthome/update_request")
UPDATE_STATUS_FILE = os.environ.get("UPDATE_STATUS_FILE", "/var/lib/smarthome/update_status.json")
# Katalog z plikami firmware serwowanymi po HTTP dla OTA (ESP pobiera stad firmware.bin).
FIRMWARE_DIR = os.environ.get("FIRMWARE_DIR", "/var/lib/smarthome/firmware")
SERVER_KIND = "system"  # Pi publikuje sie jako home/system/server/*

HEALTH_INTERVAL_S = 30  # jak czesto publikujemy zdrowie Pi (jak ESP diag)

# ---------------- Stan wspoldzielony ----------------
_lock = threading.Lock()
# nodes[kind] = {info, availability, last_diag, diag, last_seen}
nodes = {}
_started = time.time()


def log(msg):
    print("[node_monitor] %s" % msg, flush=True)


def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {}


# ---------------- Zbieranie metryk Pi ----------------
def cpu_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None


def mem_info():
    """Zwraca (total_mb, available_mb)."""
    try:
        data = {}
        with open("/proc/meminfo") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    data[parts[0].rstrip(":")] = int(parts[1])  # kB
        total = data.get("MemTotal", 0) // 1024
        avail = data.get("MemAvailable", 0) // 1024
        return total, avail
    except Exception:
        return None, None


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
        ip = out.stdout.strip().splitlines()
        return ip[0] if ip else None
    except Exception:
        return None


def server_health():
    total, avail = mem_info()
    return {
        "uptime_s": int(time.time() - _started),
        "host_uptime_s": int(read_host_uptime()),
        "cpu_temp_c": cpu_temp_c(),
        "mem_total_mb": total,
        "mem_avail_mb": avail,
        "ip": primary_ip(),
        "tailscale_ip": tailscale_ip(),
        "ts": now_iso(),
    }


def read_host_uptime():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except Exception:
        return 0.0


# ---------------- Logika rejestru wezlow ----------------
def kind_from_topic(topic):
    # home/<kind>/<leaf>  ->  <kind>
    parts = topic.split("/")
    if len(parts) >= 3 and parts[0] == "home":
        return parts[1], parts[2]
    return None, None


def evaluate_status(entry):
    """Wyznacz status wezla: online / stale / offline na podstawie availability + wieku diag."""
    avail = entry.get("availability")
    if avail == "OFFLINE":
        return "offline"
    last = entry.get("last_diag")
    if last is None:
        # Mamy availability ONLINE/info, ale brak heartbeatu jeszcze.
        return "online" if avail == "ONLINE" else "unknown"
    age = time.time() - last
    if age > DIAG_STALE_S:
        return "stale"
    return "online"


def snapshot():
    """Zbuduj pelny obraz: znane wezly + brakujace z EXPECTED_KINDS + serwer."""
    with _lock:
        result = {}
        for kind, entry in nodes.items():
            e = dict(entry)
            e["status"] = evaluate_status(entry)
            e["diag_age_s"] = (int(time.time() - entry["last_diag"])
                               if entry.get("last_diag") else None)
            result[kind] = e
        # Wezly oczekiwane, ktorych nigdy nie widzielismy.
        for kind in EXPECTED_KINDS:
            if kind not in result:
                result[kind] = {"status": "missing", "availability": None,
                                "diag": {}, "diag_age_s": None}
    return {
        "generated_at": now_iso(),
        "expected_kinds": EXPECTED_KINDS,
        "server": server_health(),
        "nodes": result,
    }


def write_state():
    try:
        snap = snapshot()
        tmp = STATE_FILE + ".tmp"
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump(snap, f, indent=2, ensure_ascii=False)
        os.replace(tmp, STATE_FILE)
    except Exception as e:
        log("Nie udalo sie zapisac %s: %s" % (STATE_FILE, e))


# ---------------- MQTT ----------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log("Polaczono z brokerem %s:%s" % (BROKER_HOST, BROKER_PORT))
        client.subscribe([("home/+/info", 0), ("home/+/availability", 0),
                          ("home/+/diag", 0)])
        publish_server_info(client)
    else:
        log("Polaczenie z brokerem nieudane (rc=%s)" % rc)


def on_message(client, userdata, msg):
    kind, leaf = kind_from_topic(msg.topic)
    if not kind or kind == SERVER_KIND:
        return  # ignoruj wlasne publikacje serwera
    payload = msg.payload.decode("utf-8", "replace").strip()
    with _lock:
        entry = nodes.setdefault(kind, {})
        entry["last_seen"] = time.time()
        if leaf == "availability":
            entry["availability"] = payload
        elif leaf == "info":
            try:
                entry["info"] = json.loads(payload)
            except Exception:
                entry["info"] = {"raw": payload}
        elif leaf == "diag":
            entry["last_diag"] = time.time()
            try:
                entry["diag"] = json.loads(payload)
            except Exception:
                entry["diag"] = {"raw": payload}
    write_state()


def publish_server_info(client):
    info = {
        "id": "rpi-" + HOSTNAME,
        "kind": SERVER_KIND,
        "name": "Smart Home Server (RPi)",
        "role": "broker+monitor",
        "host": HOSTNAME,
    }
    client.publish("home/%s/server/info" % SERVER_KIND, json.dumps(info),
                   qos=0, retain=True)
    client.publish("home/%s/server/availability" % SERVER_KIND, "ONLINE",
                   qos=0, retain=True)


def health_loop(client):
    while True:
        try:
            client.publish("home/%s/server/diag" % SERVER_KIND,
                           json.dumps(server_health()), qos=0)
            write_state()
        except Exception as e:
            log("health_loop blad: %s" % e)
        time.sleep(HEALTH_INTERVAL_S)


# ---------------- Web status ----------------
DASH_CSS = """
body{font-family:system-ui,sans-serif;margin:0;background:#0f1620;color:#e6edf3}
header{padding:16px 20px;background:#161e29;border-bottom:1px solid #2a3441}
h1{margin:0;font-size:18px}.sub{color:#8b98a5;font-size:13px;margin-top:4px}
.wrap{padding:20px;display:grid;gap:14px;grid-template-columns:repeat(auto-fill,minmax(260px,1fr))}
.card{background:#161e29;border:1px solid #2a3441;border-radius:10px;padding:14px}
.card h2{margin:0 0 8px;font-size:15px;display:flex;justify-content:space-between;align-items:center}
.badge{font-size:11px;padding:3px 8px;border-radius:20px;font-weight:600}
.online{background:#16361f;color:#3fb950}.stale{background:#3a2e10;color:#d29922}
.offline,.missing{background:#3a1717;color:#f85149}.unknown{background:#22303d;color:#8b98a5}
.row{display:flex;justify-content:space-between;font-size:13px;padding:2px 0;color:#b6c2cf}
.row b{color:#e6edf3;font-weight:500}.foot{padding:0 20px 20px;color:#6b7681;font-size:12px}
.server{border-color:#2f6feb55}
"""

DASH_HTML = """<!DOCTYPE html><html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="10"><title>Smart Home – status</title>
<style>%(css)s</style></head><body>
<header><h1>Smart Home – status wezlow</h1>
<div class="sub">%(host)s &middot; odswiezono %(ts)s &middot; auto-refresh 10s</div></header>
<div class="wrap">%(cards)s</div>
<div class="foot">JSON: <a style="color:#58a6ff" href="/api/nodes">/api/nodes</a> &middot; health: /healthz</div>
<script>
function doUpdate(){
 if(!confirm('Pobrac i zainstalowac aktualizacje serwera? Pi wykona git pull i ponowny setup.'))return;
 fetch('/update',{method:'POST'}).then(function(r){return r.json();}).then(function(d){
  alert(d.triggered?'Aktualizacja uruchomiona. Odswiez strone za okolo minute.':('Niedostepne: '+(d.error||'?')));
 }).catch(function(e){alert('Blad: '+e);});
}
</script>
</body></html>"""


def render_card(title, badge, rows, extra_class="", extra_html=""):
    rows_html = "".join(
        '<div class="row"><span>%s</span><b>%s</b></div>' % (k, v) for k, v in rows)
    return ('<div class="card %s"><h2>%s<span class="badge %s">%s</span></h2>%s%s</div>'
            % (extra_class, title, badge, badge.upper(), rows_html, extra_html))


def render_dashboard():
    snap = snapshot()
    cards = []
    # Karta serwera
    s = snap["server"]
    srv_rows = [
        ("IP", s.get("ip") or "-"),
        ("Tailscale", s.get("tailscale_ip") or "-"),
        ("Temp CPU", ("%s C" % s["cpu_temp_c"]) if s.get("cpu_temp_c") else "-"),
        ("RAM wolny", ("%s/%s MB" % (s.get("mem_avail_mb"), s.get("mem_total_mb")))),
        ("Uptime", "%s s" % s.get("host_uptime_s")),
    ]
    # Wersja serwera + wynik ostatniej aktualizacji (z pliku statusu pisanego przez 'smarthome update').
    upd = read_json(UPDATE_STATUS_FILE)
    if upd:
        ver = upd.get("after") or upd.get("before") or "-"
        srv_rows.append(("Wersja", ver))
        if upd.get("state") == "running":
            srv_rows.append(("Aktualizacja", "w toku..."))
        elif upd.get("state") == "done":
            res = "OK" if upd.get("ok") else "BLAD"
            res += " (nowa wersja)" if upd.get("changed") else " (bez zmian)"
            srv_rows.append(("Ost. update", res))
    update_btn = ""
    if WEB_UPDATE_ENABLE:
        update_btn = ('<button onclick="doUpdate()" style="margin-top:10px;width:100%;'
                      'padding:9px;background:#2f6feb;color:#fff;border:0;border-radius:6px;'
                      'cursor:pointer;font-size:13px">Sprawdz i zainstaluj aktualizacje</button>')
    cards.append(render_card("SERWER (Pi)", "online", srv_rows, "server", update_btn))
    # Karty wezlow ESP
    for kind in sorted(snap["nodes"].keys()):
        n = snap["nodes"][kind]
        diag = n.get("diag") or {}
        rows = [
            ("availability", n.get("availability") or "-"),
            ("ostatni diag", ("%s s temu" % n["diag_age_s"]) if n.get("diag_age_s") is not None else "-"),
            ("RSSI", diag.get("rssi", "-")),
            ("uptime", ("%s s" % diag["uptime_s"]) if "uptime_s" in diag else "-"),
            ("heap", diag.get("free_heap", diag.get("heap", "-"))),
        ]
        info = n.get("info") or {}
        if info.get("id"):
            rows.insert(0, ("id", info["id"]))
        cards.append(render_card(kind.upper(), n["status"], rows))
    return DASH_HTML % {
        "css": DASH_CSS, "host": HOSTNAME,
        "ts": snap["generated_at"], "cards": "".join(cards),
    }


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="text/html; charset=utf-8"):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/api/nodes"):
            self._send(200, json.dumps(snapshot(), ensure_ascii=False, indent=2),
                       "application/json; charset=utf-8")
        elif self.path.startswith("/api/update-status"):
            self._send(200, json.dumps(read_json(UPDATE_STATUS_FILE), ensure_ascii=False),
                       "application/json; charset=utf-8")
        elif self.path.startswith("/healthz"):
            self._send(200, "ok\n", "text/plain")
        elif self.path.startswith("/firmware/"):
            self._serve_firmware()
        elif self.path == "/" or self.path.startswith("/index"):
            self._send(200, render_dashboard())
        else:
            self._send(404, "not found\n", "text/plain")

    def do_POST(self):
        # Pochlon ewentualne cialo zadania (zeby keep-alive nie zglupial).
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
            if length:
                self.rfile.read(length)
        except (ValueError, TypeError):
            pass

        if self.path.rstrip("/") == "/update":
            if not WEB_UPDATE_ENABLE:
                self._send(403, json.dumps({"triggered": False, "error": "wylaczone (WEB_UPDATE_ENABLE=0)"}),
                           "application/json; charset=utf-8")
                return
            try:
                # Web dziala jako user 'smarthome' (wlasciciel STATE_DIR) – moze utworzyc wyzwalacz.
                # Rootowa .path-unit wykryje plik i odpali 'smarthome update'.
                os.makedirs(os.path.dirname(UPDATE_REQUEST_FILE), exist_ok=True)
                with open(UPDATE_REQUEST_FILE, "w") as f:
                    f.write(now_iso())
                log("Zlecono aktualizacje z dashboardu (utworzono %s)" % UPDATE_REQUEST_FILE)
                self._send(200, json.dumps({"triggered": True}), "application/json; charset=utf-8")
            except Exception as e:
                self._send(500, json.dumps({"triggered": False, "error": str(e)}),
                           "application/json; charset=utf-8")
        else:
            self._send(404, "not found\n", "text/plain")

    def _serve_firmware(self):
        # Serwuje plik .bin dla OTA. Zabezpieczone przed path traversal.
        name = os.path.basename(self.path[len("/firmware/"):].split("?", 1)[0])
        path = os.path.join(FIRMWARE_DIR, name)
        if not name or not os.path.isfile(path):
            self._send(404, "firmware not found\n", "text/plain")
            return
        try:
            size = os.path.getsize(path)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(size))
            self.end_headers()
            with open(path, "rb") as f:
                while True:
                    chunk = f.read(8192)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except Exception as e:
            log("Blad serwowania firmware %s: %s" % (name, e))

    def log_message(self, *args):
        pass  # cisza – nie zasmiecaj journala


def start_web():
    srv = ThreadingHTTPServer(("0.0.0.0", WEB_PORT), Handler)
    log("Web status na http://0.0.0.0:%s" % WEB_PORT)
    srv.serve_forever()


# ---------------- main ----------------
def main():
    client = mqtt.Client(client_id="smarthome-monitor")
    client.username_pw_set(BROKER_USER, BROKER_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    # LWT: gdy monitor padnie, serwer oznaczany OFFLINE.
    client.will_set("home/%s/server/availability" % SERVER_KIND, "OFFLINE",
                    qos=0, retain=True)

    if WEB_ENABLE:
        threading.Thread(target=start_web, daemon=True).start()

    while True:
        try:
            client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
            break
        except Exception as e:
            log("Broker niedostepny (%s) – ponawiam za 5s." % e)
            time.sleep(5)

    threading.Thread(target=health_loop, args=(client,), daemon=True).start()
    client.loop_forever()


if __name__ == "__main__":
    main()
