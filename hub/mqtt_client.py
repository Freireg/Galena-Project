import json
import time

import paho.mqtt.client as mqtt

from hub import state
from hub.config import settings


class MqttClient:
    def __init__(self):
        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self.connected = False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            client.subscribe(settings.mqtt_topic)

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False

    def _on_message(self, client, userdata, msg):
        try:
            node_id = msg.topic.split("/")[-1]
            payload = json.loads(msg.payload.decode())
            payload["_ts"] = time.time()
            state.nodes[node_id] = payload
        except Exception:
            pass

    def start(self):
        self._client.connect_async(settings.mqtt_host, settings.mqtt_port)
        self._client.loop_start()

    def stop(self):
        self._client.loop_stop()
        self._client.disconnect()
