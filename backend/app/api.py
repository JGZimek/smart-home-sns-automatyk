from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware

from app.database import get_connection, init_db
from app.repository import (
    get_latest_readings,
    get_latest_readings_per_sensor,
    get_devices,
    get_rooms,
)

app = FastAPI(
    title="Smart Home Backend",
    description="API do odczytu danych z systemu Smart Home",
    version="0.1.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
def startup():
    init_db()


@app.get("/health", tags=["system"])
def health():
    return {
        "status": "ok"
    }


@app.get("/readings", tags=["readings"])
def readings(limit: int = Query(default=50, ge=1, le=500)):
    with get_connection() as conn:
        data = get_latest_readings(conn, limit)

    return {
        "count": len(data),
        "items": data
    }


@app.get("/readings/latest", tags=["readings"])
def latest_readings():
    with get_connection() as conn:
        data = get_latest_readings_per_sensor(conn)

    return {
        "count": len(data),
        "items": data
    }


@app.get("/devices", tags=["devices"])
def devices():
    with get_connection() as conn:
        data = get_devices(conn)

    return {
        "count": len(data),
        "items": data
    }


@app.get("/rooms", tags=["rooms"])
def rooms():
    with get_connection() as conn:
        data = get_rooms(conn)

    return {
        "count": len(data),
        "items": data
    }