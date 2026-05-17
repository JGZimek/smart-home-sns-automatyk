import json
from typing import Callable

from pydantic import ValidationError

from app.models import SensorReading


class InvalidPayloadError(Exception):
    pass


def parse_sensor_payload(payload: bytes | str) -> SensorReading:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")

    try:
        data = json.loads(payload)
    except json.JSONDecodeError as error:
        raise InvalidPayloadError(f"Payload is not valid JSON: {error}") from error

    try:
        return SensorReading.model_validate(data)
    except ValidationError as error:
        raise InvalidPayloadError(f"Payload validation failed: {error}") from error


def process_mqtt_message(
    topic: str,
    payload: bytes | str,
    save_function: Callable[[SensorReading, str], int],
) -> int:
    reading = parse_sensor_payload(payload)
    reading_id = save_function(reading, topic)
    return reading_id