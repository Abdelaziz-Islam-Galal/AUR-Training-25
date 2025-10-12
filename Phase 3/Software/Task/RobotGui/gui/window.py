from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QWidget, QMainWindow, QVBoxLayout
from RobotGui.gui.camera_display import CameraDisplay
from RobotGui.gui.coordinates_display import CoordinatesDisplay

from RobotGui.core.comm.client import setup
class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 480) # fixed window size in pixels I think

        self.setCentralWidget(CentralWidget()) # where to show the captured images

        self.show() # window containing the camera display appear

    def keyPressEvent(self, event: QKeyEvent) -> None:
        
        return super().keyPressEvent(event)

class CentralWidget(QWidget):
    def __init__(self, parent :QWidget|None = None):
        super().__init__(parent)

        self._layout = QVBoxLayout(self)
        self._camera_widget = CameraDisplay() # initialising the object that will display what the camera will capture
        self._layout.addWidget(self._camera_widget)

        self._coordinates_widget = CoordinatesDisplay()
        setup(self._coordinates_widget.update_coordinates)
        self._layout.addWidget(self._coordinates_widget)