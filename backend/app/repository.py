from app.models import SensorReading


def save_sensor_reading(conn, reading: SensorReading, topic: str) -> int:
    with conn.cursor() as cur:
        cur.execute(
            """
            INSERT INTO sensor_readings (
                device_id,
                sensor_type,
                value,
                unit,
                room,
                topic,
                measured_at
            )
            VALUES (%s, %s, %s, %s, %s, %s, %s)
            RETURNING id;
            """,
            (
                reading.device_id,
                reading.sensor_type,
                reading.value,
                reading.unit,
                reading.room,
                topic,
                reading.measured_at,
            ),
        )

        new_id = cur.fetchone()[0]

    conn.commit()
    return new_id


def get_latest_readings(conn, limit: int = 50):
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT
                id,
                device_id,
                sensor_type,
                value,
                unit,
                room,
                topic,
                measured_at,
                created_at
            FROM sensor_readings
            ORDER BY created_at DESC
            LIMIT %s;
            """,
            (limit,),
        )

        rows = cur.fetchall()

    return [
        {
            "id": row[0],
            "device_id": row[1],
            "sensor_type": row[2],
            "value": row[3],
            "unit": row[4],
            "room": row[5],
            "topic": row[6],
            "measured_at": row[7],
            "created_at": row[8],
        }
        for row in rows
    ]


def get_latest_readings_per_sensor(conn):
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT DISTINCT ON (device_id, sensor_type)
                id,
                device_id,
                sensor_type,
                value,
                unit,
                room,
                topic,
                measured_at,
                created_at
            FROM sensor_readings
            ORDER BY device_id, sensor_type, created_at DESC;
            """
        )

        rows = cur.fetchall()

    return [
        {
            "id": row[0],
            "device_id": row[1],
            "sensor_type": row[2],
            "value": row[3],
            "unit": row[4],
            "room": row[5],
            "topic": row[6],
            "measured_at": row[7],
            "created_at": row[8],
        }
        for row in rows
    ]


def get_devices(conn):
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT
                device_id,
                COUNT(*) AS readings_count,
                MAX(created_at) AS last_seen
            FROM sensor_readings
            GROUP BY device_id
            ORDER BY device_id;
            """
        )

        rows = cur.fetchall()

    return [
        {
            "device_id": row[0],
            "readings_count": row[1],
            "last_seen": row[2],
        }
        for row in rows
    ]


def get_rooms(conn):
    with conn.cursor() as cur:
        cur.execute(
            """
            SELECT
                room,
                COUNT(*) AS readings_count,
                MAX(created_at) AS last_seen
            FROM sensor_readings
            WHERE room IS NOT NULL
            GROUP BY room
            ORDER BY room;
            """
        )

        rows = cur.fetchall()

    return [
        {
            "room": row[0],
            "readings_count": row[1],
            "last_seen": row[2],
        }
        for row in rows
    ]