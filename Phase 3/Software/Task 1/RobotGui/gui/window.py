from PySide6.QtWidgets import QWidget, QMainWindow
from PySide6.QtCore import QTimer
from RobotGui.gui.camera_display import CameraDisplay

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setFixedSize(640, 480) # fixed window size in pixels I think

        self._camera_widget = CameraDisplay(self) # initialising the object that will display what the camera will capture
        self.setCentralWidget(self._camera_widget) # where to show the captured images

        self.show() # window containing the camera display appear

        # Timer to update frames (makes it like a video)
        self._camera_timer = QTimer()
        self._camera_timer.timeout.connect(self._camera_widget.update_view)
        self._camera_timer.setInterval(50)
        self._camera_timer.start()