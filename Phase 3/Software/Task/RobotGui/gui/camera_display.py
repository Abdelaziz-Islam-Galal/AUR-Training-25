from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy
from PySide6.QtGui import QImage, QPixmap, QResizeEvent
from PySide6.QtCore import QTimer
from RobotGui.core.cv import Camera

class CameraDisplay(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        
        self._camera_device = Camera()

        self._frame_view = QLabel(self)
        self._frame_view.setScaledContents(True)

        self.update_view()

        # Timer to update frames (makes it like a video)
        self._camera_timer = QTimer()
        self._camera_timer.timeout.connect(self.update_view)
        self._camera_timer.setInterval(50)
        self._camera_timer.start()

    def resizeEvent(self, event: QResizeEvent) -> None: 
        super().resizeEvent(event)
        # that is a function in the QWidget Class, I overwrite it so the CameraDisplay window
        # is dynamic and gets resized with the main window

        self._frame_view.resize(event.size())
        # I am giving the _frame_view parameter the size of the event where the event is the main window that I resize

    def update_view(self):
        frame = self._camera_device.frame
        image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888) # type: ignore
        self._frame_view.setPixmap(QPixmap.fromImage(image))