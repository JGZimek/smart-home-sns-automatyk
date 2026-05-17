from fastapi.testclient import TestClient

import app.api as api_module


client = TestClient(api_module.app)


def test_health_endpoint():
    response = client.get("/health")

    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_readings_endpoint(monkeypatch):
    fake_data = [
        {
            "id": 1,
            "device_id": "esp32_01",
            "sensor_type": "temperature",
            "value": 23.6,
            "unit": "C",
            "room": "livingroom",
            "topic": "home/livingroom/temperature",
            "measured_at": None,
            "created_at": "2026-05-17T12:30:00",
        }
    ]

    class FakeConnection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            pass

    def fake_get_connection():
        return FakeConnection()

    def fake_get_latest_readings(conn, limit):
        assert limit == 50
        return fake_data

    monkeypatch.setattr(api_module, "get_connection", fake_get_connection)
    monkeypatch.setattr(api_module, "get_latest_readings", fake_get_latest_readings)

    response = client.get("/readings")

    assert response.status_code == 200
    assert response.json() == {
        "count": 1,
        "items": fake_data,
    }


def test_readings_endpoint_with_limit(monkeypatch):
    fake_data = [
        {
            "id": 1,
            "device_id": "esp32_01",
            "sensor_type": "temperature",
            "value": 23.6,
            "unit": "C",
            "room": "livingroom",
            "topic": "home/livingroom/temperature",
            "measured_at": None,
            "created_at": "2026-05-17T12:30:00",
        },
        {
            "id": 2,
            "device_id": "esp32_01",
            "sensor_type": "humidity",
            "value": 58,
            "unit": "%",
            "room": "livingroom",
            "topic": "home/livingroom/humidity",
            "measured_at": None,
            "created_at": "2026-05-17T12:31:00",
        },
    ]

    class FakeConnection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            pass

    def fake_get_connection():
        return FakeConnection()

    def fake_get_latest_readings(conn, limit):
        assert limit == 2
        return fake_data

    monkeypatch.setattr(api_module, "get_connection", fake_get_connection)
    monkeypatch.setattr(api_module, "get_latest_readings", fake_get_latest_readings)

    response = client.get("/readings?limit=2")

    assert response.status_code == 200
    assert response.json()["count"] == 2
    assert response.json()["items"] == fake_data


def test_latest_readings_endpoint(monkeypatch):
    fake_data = [
        {
            "id": 3,
            "device_id": "esp32_01",
            "sensor_type": "temperature",
            "value": 24.1,
            "unit": "C",
            "room": "livingroom",
            "topic": "home/livingroom/temperature",
            "measured_at": None,
            "created_at": "2026-05-17T12:35:00",
        }
    ]

    class FakeConnection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            pass

    def fake_get_connection():
        return FakeConnection()

    def fake_get_latest_readings_per_sensor(conn):
        return fake_data

    monkeypatch.setattr(api_module, "get_connection", fake_get_connection)
    monkeypatch.setattr(
        api_module,
        "get_latest_readings_per_sensor",
        fake_get_latest_readings_per_sensor,
    )

    response = client.get("/readings/latest")

    assert response.status_code == 200
    assert response.json() == {
        "count": 1,
        "items": fake_data,
    }


def test_devices_endpoint(monkeypatch):
    fake_data = [
        {
            "device_id": "esp32_01",
            "readings_count": 10,
            "last_seen": "2026-05-17T12:35:00",
        }
    ]

    class FakeConnection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            pass

    def fake_get_connection():
        return FakeConnection()

    def fake_get_devices(conn):
        return fake_data

    monkeypatch.setattr(api_module, "get_connection", fake_get_connection)
    monkeypatch.setattr(api_module, "get_devices", fake_get_devices)

    response = client.get("/devices")

    assert response.status_code == 200
    assert response.json() == {
        "count": 1,
        "items": fake_data,
    }


def test_rooms_endpoint(monkeypatch):
    fake_data = [
        {
            "room": "livingroom",
            "readings_count": 10,
            "last_seen": "2026-05-17T12:35:00",
        }
    ]

    class FakeConnection:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            pass

    def fake_get_connection():
        return FakeConnection()

    def fake_get_rooms(conn):
        return fake_data

    monkeypatch.setattr(api_module, "get_connection", fake_get_connection)
    monkeypatch.setattr(api_module, "get_rooms", fake_get_rooms)

    response = client.get("/rooms")

    assert response.status_code == 200
    assert response.json() == {
        "count": 1,
        "items": fake_data,
    }