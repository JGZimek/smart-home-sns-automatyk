# Smart Home Backend

Backend w Pythonie do projektu Smart Home. Odbiera dane z MQTT, zapisuje je do PostgreSQL i udostępnia przez REST API w FastAPI.

## Technologie

- Python
- FastAPI
- PostgreSQL
- Mosquitto MQTT
- pytest

## Wymagania

- Python 3.11+
- PostgreSQL
- Mosquitto MQTT Broker

## Konfiguracja

Wejdź do folderu backendu:

```powershell
cd backend
```

Utwórz i aktywuj środowisko:

```powershell
python -m venv .venv
.venv\Scripts\activate
```

Zainstaluj zależności:

```powershell
pip install -r requirements.txt
```

Utwórz bazę PostgreSQL:

```sql
CREATE DATABASE smarthome;
```

Utwórz plik `.env` w folderze `backend/`:

```env
DATABASE_URL=postgresql://postgres:postgres@localhost:5432/smarthome

MQTT_HOST=localhost
MQTT_PORT=1883
MQTT_TOPIC=home/#
MQTT_CLIENT_ID=smarthome-backend
```

Jeśli hasło do PostgreSQL jest inne niż `postgres`, zmień je w `DATABASE_URL`.

## Uruchomienie

### 1. Mosquitto

```powershell
cd "C:\Program Files\mosquitto"
.\mosquitto.exe -v
```

### 2. MQTT Worker

```powershell
cd backend
.venv\Scripts\activate
python -m app.mqtt_worker
```

### 3. API

```powershell
cd backend
.venv\Scripts\activate
uvicorn app.api:app --reload
```

API:

```text
http://localhost:8000
```

Swagger:

```text
http://localhost:8000/docs
```

Aby udostępnić API w LAN/VPN:

```powershell
uvicorn app.api:app --host 0.0.0.0 --port 8000 --reload
```

Wtedy frontend korzysta z:

```text
http://IP_BACKENDU:8000
```

## Endpointy API

```http
GET /health
```

Sprawdza, czy API działa.

```http
GET /readings
```

Zwraca ostatnie pomiary.

```http
GET /readings?limit=10
```

Zwraca określoną liczbę ostatnich pomiarów.

```http
GET /readings/latest
```

Zwraca najnowszy pomiar dla każdego czujnika.

```http
GET /devices
```

Zwraca listę urządzeń.

```http
GET /rooms
```

Zwraca listę pomieszczeń.

## Przykładowa odpowiedź `/readings`

```json
{
  "count": 1,
  "items": [
    {
      "id": 1,
      "device_id": "esp32_01",
      "sensor_type": "temperature",
      "value": 23.6,
      "unit": "C",
      "room": "livingroom",
      "topic": "home/livingroom/temperature",
      "measured_at": null,
      "created_at": "2026-05-17T12:30:00"
    }
  ]
}
```

## Testowe wysłanie danych MQTT

```powershell
cd "C:\Program Files\mosquitto"

.\mosquitto_pub.exe -h localhost -p 1883 -t "home/livingroom/temperature" -m "{`"device_id`":`"esp32_01`",`"sensor`":`"temperature`",`"value`":23.6,`"unit`":`"C`",`"room`":`"livingroom`"}"
```

Po wysłaniu dane powinny być dostępne pod:

```text
http://localhost:8000/readings
```

## Testy

```powershell
cd backend
.venv\Scripts\activate
python -m pytest -v
```
