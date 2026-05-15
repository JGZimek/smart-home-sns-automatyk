import time
import socket
import fcntl
import struct
import sys
from RPLCD.i2c import CharLCD

# --- KONFIGURACJA ---
I2C_ADDRESS = 0x27
CHECK_INTERVAL = 10  # Sprawdzaj co 10 sekund (żeby szybciej reagować na padnięcie brokera)

def get_ip_address(ifname):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode('utf-8'))
        )[20:24])
    except:
        return None

def check_broker(ip, port=1883):
    """Sprawdza, czy port brokera MQTT nasłuchuje"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    try:
        result = sock.connect_ex((ip, port))
        sock.close()
        return result == 0
    except:
        return False

def main():
    try:
        lcd = CharLCD(i2c_expander='PCF8574', address=I2C_ADDRESS, port=1,
                      cols=16, rows=2, dotsize=8,
                      charmap='A00', auto_linebreaks=True, backlight_enabled=True)
    except Exception as e:
        print(f"Błąd I2C: {e}")
        sys.exit(1)

    lcd.clear()
    lcd.write_string('SmartHome OS')
    lcd.cursor_pos = (1, 0)
    lcd.write_string('Bootowanie...')
    
    last_state = ""

    while True:
        # Sprawdzanie trybu awaryjnego AP
        ap_ip = get_ip_address('wlan0')
        if ap_ip == '10.42.0.1':
            current_state = f"TRYB AWARYJNY\nAP: SmartHome-CFG"
        else:
            current_ip = ap_ip if ap_ip else get_ip_address('eth0')
            
            if not current_ip:
                current_state = "Blad Sieci...\nBrak WiFi/LAN"
            else:
                # Test czy Mosquitto (Docker) żyje
                broker_ok = check_broker('127.0.0.1')
                status_txt = "Broker: OK" if broker_ok else "Broker: ERROR!"
                current_state = f"IP:{current_ip}\n{status_txt}"

        if current_state != last_state:
            try:
                lcd.clear()
                lines = current_state.split('\n')
                lcd.write_string(lines[0])
                if len(lines) > 1:
                    lcd.cursor_pos = (1, 0)
                    lcd.write_string(lines[1])
                last_state = current_state
            except Exception:
                pass

        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
