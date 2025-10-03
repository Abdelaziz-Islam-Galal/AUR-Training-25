from PySide6.QtWidgets import QWidget, QLabel, QSizePolicy
from PySide6.QtGui import QImage, QPixmap
from RobotGui.core.cv import Camera

class CameraDisplay(QWidget):
    def __init__(self, parent: QWidget):
        super().__init__(parent)
        
        self._camera_device = Camera()

        self._frame_view = QLabel(self)
        self._frame_view.setScaledContents(True)

        self.update_view()

    def update_view(self):
        frame = self._camera_device.frame
        image = QImage(frame.data, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format.Format_BGR888)
        self._frame_view.setPixmap(QPixmap.fromImage(image))