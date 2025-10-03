from PySide6.QtWidgets import QWidget, QMainWindow
from PySide6.QtCore import QTimer
from RobotGui.gui.camera_display import CameraDisplay

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setFixedSize(640, 480)

        self._camera_widget = CameraDisplay(self)
        self.setCentralWidget(self._camera_widget)

        self.show()

        self._camera_timer = QTimer()
        self._camera_timer.timeout.connect(self._camera_widget.update_view)
        self._camera_timer.setInterval(500)
        self._camera_timer.start()