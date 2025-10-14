#from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtGui import Qt
from RobotGui.gui.coordinates_display import CoordinatesDisplay as coords
from RobotGui.core.comm.client import Mqtt  

class MovementPublish():
    def __init__(self, mqtt_client):
        print("entered MovementPublish init")
        self.mqtt = mqtt_client
        self.x, self.y = 0, 0

    def handle_key_event(self, key_event):
        key = key_event.key()
        
        if key == Qt.Key.Key_Left:
            self.x -= 1
        elif key == Qt.Key.Key_Right:
            self.x += 1
        elif key == Qt.Key.Key_Up:
            self.y += 1
        elif key == Qt.Key.Key_Down:
            self.y -= 1
        else:
            return

        self.publish_movement()

    def publish_movement(self):
        msg = self.mqtt._mqttc_pub.publish("robot/movement", f'{self.x},{self.y}')
        if self.mqtt.unacked_publish is not None:
            self.mqtt.unacked_publish.add(msg.mid)
        msg.wait_for_publish()
        print(f"Published movement: {self.x}, {self.y}")
    
    