import pytest
from pydantic import ValidationError

from app.models import SensorReading


def test_valid_sensor_reading():
    payload = {
        "device_id": "esp32_01",
        "sensor": "temperature",
        "value": 23.5,
        "unit": "C",
        "room": "livingroom",
    }

    reading = SensorReading.model_validate(payload)

    assert reading.device_id == "esp32_01"
    assert reading.sensor_type == "temperature"
    assert reading.value == 23.5
    assert reading.unit == "C"
    assert reading.room == "livingroom"


def test_missing_device_id_is_invalid():
    payload = {
        "sensor": "temperature",
        "value": 23.5,
    }

    with pytest.raises(ValidationError):
        SensorReading.model_validate(payload)


def test_missing_sensor_is_invalid():
    payload = {
        "device_id": "esp32_01",
        "value": 23.5,
    }

    with pytest.raises(ValidationError):
        SensorReading.model_validate(payload)


def test_value_must_be_number():
    payload = {
        "device_id": "esp32_01",
        "sensor": "temperature",
        "value": "abc",
    }

    with pytest.raises(ValidationError):
        SensorReading.model_validate(payload)


def test_extra_fields_are_ignored():
    payload = {
        "device_id": "esp32_01",
        "sensor": "temperature",
        "value": 22.1,
        "some_extra_field": "ignored",
    }

    reading = SensorReading.model_validate(payload)

    assert reading.device_id == "esp32_01"
    assert reading.sensor_type == "temperature"
    assert reading.value == 22.1
    assert not hasattr(reading, "some_extra_field")