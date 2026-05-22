from amqtt.broker import Broker

_CONFIG = {
    "listeners": {
        "default": {
            "type": "tcp",
            "bind": "0.0.0.0:1883",
        },
    },
    "sys_interval": 0,
}

_broker: Broker | None = None


async def start():
    global _broker
    _broker = Broker(_CONFIG)
    await _broker.start()


async def stop():
    if _broker:
        await _broker.stop()
