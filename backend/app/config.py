from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    database_url: str

    mqtt_host: str = "localhost"
    mqtt_port: int = 1883
    mqtt_topic: str = "home/#"
    mqtt_client_id: str = "smarthome-backend"

    class Config:
        env_file = ".env"


settings = Settings()