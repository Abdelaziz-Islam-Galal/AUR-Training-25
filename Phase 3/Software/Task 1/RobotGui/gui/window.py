from PySide6.QtWidgets import QWidget, QMainWindow
from RobotGui.gui.camera_display import CameraDisplay

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 480) # fixed window size in pixels I think

        self._camera_widget = CameraDisplay(self) # initialising the object that will display what the camera will capture
        self.setCentralWidget(self._camera_widget) # where to show the captured images

        self.show() # window containing the camera display appear

        