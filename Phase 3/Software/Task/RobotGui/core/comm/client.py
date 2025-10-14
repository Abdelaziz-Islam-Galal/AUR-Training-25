from paho.mqtt.client import Client as MC
from paho.mqtt.enums import CallbackAPIVersion
import paho.mqtt.subscribe as subscribe
import RobotGui.core.comm.pub.movment as publish
import time
# import paho.mqtt.publish as publish

import RobotGui.core.comm.sub.coords as coords

class Mqtt():
    _mqttc_sub = MC(CallbackAPIVersion.VERSION2)
    _mqttc_pub = MC(CallbackAPIVersion.VERSION2)

    def _on_publish(self, client, userdata, mid, reason_code, properties):
        print("publishing action is starting")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print('Failed to connect. Retrying..')
        else:
            subscribe.callback(coords.callback, 'robot/coordinates')

    unacked_publish = None
    def setup(self, coords_slot, address = 'localhost', port = 1883):
        coords.slot = coords_slot
        self._mqttc_sub.on_connect = self._on_connect
        self._mqttc_sub.connect(address, port)
        self._mqttc_sub.loop_start()

        self.unacked_publish = set()
        self._mqttc_pub.on_publish = self._on_publish

        self._mqttc_pub.user_data_set(self.unacked_publish)
        self._mqttc_pub.connect("localhost", 1883)
        self._mqttc_pub.loop_start()
