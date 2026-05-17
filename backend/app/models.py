from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field, ConfigDict


class SensorReading(BaseModel):
    model_config = ConfigDict(
        populate_by_name=True,
        extra="ignore"
    )

    device_id: str = Field(min_length=1)
    sensor_type: str = Field(alias="sensor", min_length=1)
    value: float
    unit: Optional[str] = None
    room: Optional[str] = None
    measured_at: Optional[datetime] = None