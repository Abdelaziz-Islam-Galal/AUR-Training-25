from PySide6.QtWidgets import QWidget, QLabel
from PySide6.QtCore import Slot

class CoordinatesDisplay(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._label = QLabel(self)
        self._label.setText('Robot is not connected.')

    @Slot() # we let QT to know that even if this function is called in another thread, still manage it in the QT's main thread anyway
    def update_coordinates(self, x: float, y: float):
        self._label.setText(f'Robot is at: {x}, {y}')
        return (x,y)
        