![](resources/GalenaLogo.png)
# Galena — Sensor Hub

Galena is an embedded Linux sensor hub that aggregates data from wireless peripheral nodes over MQTT, reads GPS over UART, fetches weather from the web, and exposes everything through a real-time web dashboard.

Designed for ARM systems with ~512 MB RAM (e.g., Toradex Verdin, Raspberry Pi, Milk-V Duo S).

---

## Architecture

```
Peripheral nodes  ──MQTT──►  Mosquitto broker
GPS module (UART) ──────────►
                              │
                        FastAPI hub (Python)
                              │
                        WebSocket push (1 Hz)
                              │
                        Browser dashboard
```

The hub runs a single FastAPI process. Background threads handle MQTT and GPS; async tasks handle weather fetches and WebSocket broadcasts.

---

## Dashboard panels

| Panel | Contents |
|-------|----------|
| **Peripheral nodes** | One widget per MQTT node, auto-created on first message. Shows all key/value fields from the JSON payload plus a freshness indicator. |
| **Diagnostics** | CPU load, core temperature, memory usage (with progress bar), uptime. |
| **GPS** | Latitude, longitude, altitude, speed, satellite count, fix type. |
| **Weather** | Current temperature, condition, wind, humidity, sunrise/sunset, 3-day forecast (Open-Meteo). |

---

## Stack

| Component | Library |
|-----------|---------|
| Web server + WebSocket | FastAPI + uvicorn |
| MQTT client | paho-mqtt |
| GPS (UART / NMEA) | pyserial + pynmea2 |
| System diagnostics | psutil |
| Weather API | httpx → Open-Meteo |
| Frontend | Vanilla JS + CSS Grid, no build step |

---

## Quick start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure

```bash
cp .env.example .env
# edit .env — set MQTT_HOST, GPS_PORT, WEATHER_LAT/LON, etc.
```

### 3. Run

```bash
uvicorn hub.main:app --host 0.0.0.0 --port 8000
```

Open `http://<device-ip>:8000` in a browser.

---

## MQTT node protocol

Peripheral nodes publish JSON to `nodes/<node_id>`:

```
Topic:   nodes/sensor-a
Payload: {"temperature": 24.3, "humidity": 61, "battery_mv": 3712}
```

Any JSON object is accepted — the dashboard renders all fields automatically.

---

## Configuration reference

| Variable | Default | Description |
|----------|---------|-------------|
| `MQTT_HOST` | `localhost` | Mosquitto broker address |
| `MQTT_PORT` | `1883` | Broker port |
| `MQTT_TOPIC` | `nodes/#` | Topic filter |
| `GPS_PORT` | `/dev/ttyS1` | Serial port for GPS module |
| `GPS_BAUD` | `9600` | GPS baud rate |
| `WEATHER_LAT` | `-23.5505` | Location latitude |
| `WEATHER_LON` | `-46.6333` | Location longitude |
| `WEATHER_INTERVAL` | `300` | Seconds between weather fetches |
