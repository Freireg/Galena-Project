from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    mqtt_host: str = "localhost"
    mqtt_port: int = 1883
    mqtt_topic: str = "nodes/#"

    gps_port: str = "/dev/ttyS1"
    gps_baud: int = 9600

    weather_lat: float = -22.909135
    weather_lon: float = -47.062301
    weather_interval: int = 300

    model_config = {"env_file": ".env"}


settings = Settings()
