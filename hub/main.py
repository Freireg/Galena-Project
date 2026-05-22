import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from hub import broker, diagnostics, state
from hub.config import settings
from hub.gps import GpsReader
from hub.mqtt_client import MqttClient
from hub.weather import WeatherFetcher, _URL, _PARAMS

_clients: set[WebSocket] = set()
_weather: WeatherFetcher | None = None


async def _broadcast(payload: dict):
    dead: set[WebSocket] = set()
    for ws in list(_clients):          # snapshot so connect/disconnect mid-send is safe
        try:
            await ws.send_json(payload)
        except Exception:
            dead.add(ws)
    _clients.difference_update(dead)


async def _broadcast_loop():
    while True:
        try:
            await _broadcast({
                "nodes": state.nodes,
                "diagnostics": state.diagnostics,
                "gps": state.gps,
                "weather": state.weather,
            })
        except Exception as exc:
            print(f"[broadcast] error: {exc}")
        await asyncio.sleep(1)


async def _diagnostics_loop():
    while True:
        state.diagnostics = diagnostics.collect()
        await asyncio.sleep(5)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _weather
    gps = GpsReader()
    _weather = WeatherFetcher()

    await broker.start()          # broker must be up before paho tries to connect
    mqtt = MqttClient()
    mqtt.start()
    gps.start()

    asyncio.create_task(_diagnostics_loop())
    asyncio.create_task(_weather.fetch_loop())
    asyncio.create_task(_broadcast_loop())

    yield

    mqtt.stop()
    gps.stop()
    await broker.stop()


app = FastAPI(title="Galena Hub", lifespan=lifespan)
app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/", response_class=HTMLResponse)
async def index():
    return Path("static/index.html").read_text()


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    _clients.add(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        _clients.discard(websocket)


@app.get("/api/debug")
async def api_debug():
    """Shows active settings and the exact URL the last weather fetch used."""
    query = {**_PARAMS, "latitude": settings.weather_lat, "longitude": settings.weather_lon}
    query_str = "&".join(f"{k}={v}" for k, v in query.items())
    return JSONResponse({
        "settings": {
            "weather_lat": settings.weather_lat,
            "weather_lon": settings.weather_lon,
            "weather_interval_s": settings.weather_interval,
            "mqtt_host": settings.mqtt_host,
            "mqtt_port": settings.mqtt_port,
            "gps_port": settings.gps_port,
        },
        "weather_api_url": f"{_URL}?{query_str}",
        "weather_state": state.weather,
    })


@app.post("/api/weather/refresh")
async def api_weather_refresh():
    """Triggers an immediate weather re-fetch outside of the normal interval."""
    if _weather:
        asyncio.create_task(_weather.refresh())
    return {"ok": True}
