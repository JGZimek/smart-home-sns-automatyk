import psycopg

from app.config import settings


def get_connection():
    return psycopg.connect(settings.database_url)


def init_db():
    with get_connection() as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                CREATE TABLE IF NOT EXISTS sensor_readings (
                    id BIGSERIAL PRIMARY KEY,
                    device_id TEXT NOT NULL,
                    sensor_type TEXT NOT NULL,
                    value DOUBLE PRECISION NOT NULL,
                    unit TEXT,
                    room TEXT,
                    topic TEXT NOT NULL,
                    measured_at TIMESTAMPTZ,
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
                );
                """
            )
        conn.commit()