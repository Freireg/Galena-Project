import time

import psutil

_BOOT_TIME = psutil.boot_time()

_TEMP_KEYS = (
    "cpu_thermal", "cpu-thermal", "soc_thermal",
    "cpu_temp", "coretemp", "k10temp",
)


def collect() -> dict:
    temps = psutil.sensors_temperatures() or {}
    cpu_temp = None
    for key in _TEMP_KEYS:
        if key in temps and temps[key]:
            cpu_temp = round(temps[key][0].current, 1)
            break

    mem = psutil.virtual_memory()
    load = psutil.getloadavg()

    return {
        "cpu_percent": psutil.cpu_percent(interval=None),
        "cpu_temp": cpu_temp,
        "mem_total_mb": round(mem.total / 1024 / 1024),
        "mem_used_mb": round(mem.used / 1024 / 1024),
        "mem_percent": round(mem.percent, 1),
        "load_1": round(load[0], 2),
        "load_5": round(load[1], 2),
        "uptime_s": int(time.time() - _BOOT_TIME),
    }
