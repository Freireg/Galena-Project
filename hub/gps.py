import threading

import pynmea2
import serial

from hub import state
from hub.config import settings


class GpsReader:
    def __init__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._running = False

    def _run(self):
        try:
            with serial.Serial(settings.gps_port, settings.gps_baud, timeout=1) as ser:
                while self._running:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line.startswith(("$GP", "$GN")):
                        try:
                            self._update(pynmea2.parse(line))
                        except pynmea2.ParseError:
                            pass
        except serial.SerialException as e:
            state.gps = {"error": str(e)}

    def _update(self, msg):
        if isinstance(msg, pynmea2.types.talker.GGA):
            state.gps.update({
                "lat": msg.latitude,
                "lon": msg.longitude,
                "altitude": float(msg.altitude) if msg.altitude else None,
                "fix": int(msg.gps_qual),
                "satellites": msg.num_sats,
            })
        elif isinstance(msg, pynmea2.types.talker.RMC):
            state.gps.update({
                "lat": msg.latitude,
                "lon": msg.longitude,
                "speed_kmh": round(msg.spd_over_grnd * 1.852, 1) if msg.spd_over_grnd else 0.0,
            })
        elif isinstance(msg, pynmea2.types.talker.VTG):
            if msg.spd_over_grnd_kmph is not None:
                state.gps.update({"speed_kmh": round(msg.spd_over_grnd_kmph, 1)})

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
