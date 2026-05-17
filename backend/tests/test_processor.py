import pytest

from app.processor import (
    parse_sensor_payload,
    process_mqtt_message,
    InvalidPayloadError,
)


def test_parse_valid_payload_from_bytes():
    payload = b'{"device_id":"esp32_01","sensor":"temperature","value":22.4,"unit":"C"}'

    reading = parse_sensor_payload(payload)

    assert reading.device_id == "esp32_01"
    assert reading.sensor_type == "temperature"
    assert reading.value == 22.4
    assert reading.unit == "C"


def test_parse_valid_payload_from_string():
    payload = '{"device_id":"esp32_01","sensor":"humidity","value":58,"unit":"%"}'

    reading = parse_sensor_payload(payload)

    assert reading.device_id == "esp32_01"
    assert reading.sensor_type == "humidity"
    assert reading.value == 58
    assert reading.unit == "%"


def test_invalid_json_raises_error():
    payload = b'{"device_id":"esp32_01",'

    with pytest.raises(InvalidPayloadError):
        parse_sensor_payload(payload)


def test_missing_required_field_raises_error():
    payload = b'{"device_id":"esp32_01","value":22.4}'

    with pytest.raises(InvalidPayloadError):
        parse_sensor_payload(payload)


def test_process_mqtt_message_calls_save_function():
    payload = b'{"device_id":"esp32_01","sensor":"humidity","value":60,"unit":"%"}'
    topic = "home/livingroom/humidity"

    saved = {}

    def fake_save_function(reading, received_topic):
        saved["device_id"] = reading.device_id
        saved["sensor_type"] = reading.sensor_type
        saved["value"] = reading.value
        saved["topic"] = received_topic
        return 123

    result = process_mqtt_message(topic, payload, fake_save_function)

    assert result == 123
    assert saved["device_id"] == "esp32_01"
    assert saved["sensor_type"] == "humidity"
    assert saved["value"] == 60
    assert saved["topic"] == "home/livingroom/humidity"