from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
from time import sleep
from random import random

_mqttc = MC(CallbackAPIVersion.VERSION2)


def _on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print('Failed to connect. Retrying..')

def setup(address = 'localhost', port = 1883):
    _mqttc.on_connect = _on_connect
    _mqttc.connect(address, port)
    _mqttc.loop_start()

setup()

while True:
    sleep(1)
    x = random()
    y = random()
    _mqttc.publish('robot/coordinates', f'{x},{y}')