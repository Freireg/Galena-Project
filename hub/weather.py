import asyncio

import httpx

from hub import state
from hub.config import settings

_URL = "https://api.open-meteo.com/v1/forecast"
_PARAMS = {
    "current": "temperature_2m,weathercode,windspeed_10m,relativehumidity_2m",
    "daily": "temperature_2m_max,temperature_2m_min,sunrise,sunset,weathercode",
    "timezone": "auto",
    "forecast_days": 3,
}

WMO_DESCRIPTIONS = {
    0: "Clear sky", 1: "Mainly clear", 2: "Partly cloudy", 3: "Overcast",
    45: "Fog", 48: "Rime fog",
    51: "Light drizzle", 53: "Drizzle", 55: "Heavy drizzle",
    61: "Light rain", 63: "Rain", 65: "Heavy rain",
    71: "Light snow", 73: "Snow", 75: "Heavy snow",
    80: "Light showers", 81: "Showers", 82: "Heavy showers",
    95: "Thunderstorm",
}


class WeatherFetcher:
    async def fetch_loop(self):
        await self._fetch()
        while True:
            await asyncio.sleep(settings.weather_interval)
            await self._fetch()

    async def refresh(self):
        await self._fetch()

    async def _fetch(self):
        params = {
            **_PARAMS,
            "latitude": settings.weather_lat,
            "longitude": settings.weather_lon,
        }
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                r = await client.get(_URL, params=params)
                r.raise_for_status()
                data = r.json()
                current = data.get("current", {})
                daily = data.get("daily", {})
                state.weather = {
                    "temp": current.get("temperature_2m"),
                    "wind_kmh": current.get("windspeed_10m"),
                    "humidity": current.get("relativehumidity_2m"),
                    "condition": WMO_DESCRIPTIONS.get(current.get("weathercode", -1), "Unknown"),
                    "sunrise": daily.get("sunrise", [None])[0],
                    "sunset": daily.get("sunset", [None])[0],
                    "forecast": [
                        {
                            "date": daily["time"][i],
                            "max": daily["temperature_2m_max"][i],
                            "min": daily["temperature_2m_min"][i],
                            "condition": WMO_DESCRIPTIONS.get(daily["weathercode"][i], ""),
                        }
                        for i in range(len(daily.get("time", [])))
                    ],
                }
        except Exception as e:
            state.weather["error"] = str(e)
