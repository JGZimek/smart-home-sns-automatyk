import time
import socket
import fcntl
import struct
import sys
from RPLCD.i2c import CharLCD

# --- KONFIGURACJA ---
I2C_ADDRESS = 0x27
CHECK_INTERVAL = 60  # Sprawdzaj IP co 60 sekund (oszczędność CPU)

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

def main():
    try:
        # Inicjalizacja LCD
        lcd = CharLCD(i2c_expander='PCF8574', address=I2C_ADDRESS, port=1,
                      cols=16, rows=2, dotsize=8,
                      charmap='A00',
                      auto_linebreaks=True,
                      backlight_enabled=True)
    except Exception as e:
        print(f"Błąd inicjalizacji LCD: {e}")
        sys.exit(1)

    lcd.clear()
    lcd.write_string('Bootowanie...')
    
    last_ip = ""

    while True:
        # Priorytet: WiFi (wlan0), potem kabel (eth0)
        current_ip = get_ip_address('wlan0')
        if not current_ip:
             current_ip = get_ip_address('eth0')
        
        if not current_ip:
            current_ip = "Brak sieci..."

        # Rysuj na ekranie TYLKO jeśli IP się zmieniło (oszczędność I2C i CPU)
        if current_ip != last_ip:
            try:
                lcd.clear()
                lcd.write_string('MQTT Broker IP:')
                lcd.cursor_pos = (1, 0)
                lcd.write_string(current_ip)
                last_ip = current_ip
            except Exception as e:
                print(f"Błąd I2C: {e}")

        # RPi Zero 2W może spać. To nie blokuje systemu.
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
