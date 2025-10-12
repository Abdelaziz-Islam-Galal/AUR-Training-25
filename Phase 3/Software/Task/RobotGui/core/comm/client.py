from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
import paho.mqtt.subscribe as subscribe

import RobotGui.core.comm.sub.coords as coords

_mqttc = MC(CallbackAPIVersion.VERSION2)


def _on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print('Failed to connect. Retrying..')
    else:
        subscribe.callback(coords.callback, 'robot/coordinates')

def setup(coords_slot, address = 'localhost', port = 1883):
    coords.slot = coords_slot
    _mqttc.on_connect = _on_connect
    _mqttc.connect(address, port)
    _mqttc.loop_start()