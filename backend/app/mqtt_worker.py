import paho.mqtt.client as mqtt

from app.config import settings
from app.database import get_connection, init_db
from app.processor import process_mqtt_message, InvalidPayloadError
from app.repository import save_sensor_reading


def main():
    init_db()

    db_conn = get_connection()

    def save_function(reading, topic):
        return save_sensor_reading(db_conn, reading, topic)

    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"Connected to MQTT broker with result code: {reason_code}")
        client.subscribe(settings.mqtt_topic)
        print(f"Subscribed to topic: {settings.mqtt_topic}")

    def on_message(client, userdata, message):
        topic = message.topic
        payload = message.payload

        print(f"Received message from topic: {topic}")

        try:
            reading_id = process_mqtt_message(topic, payload, save_function)
            print(f"Saved reading with id: {reading_id}")

        except InvalidPayloadError as error:
            print(f"Invalid payload: {error}")

        except Exception as error:
            print(f"Unexpected error: {error}")

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=settings.mqtt_client_id,
    )

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(settings.mqtt_host, settings.mqtt_port)

    print("MQTT worker started")
    client.loop_forever()


if __name__ == "__main__":
    main()