import cv2
from pathlib import Path

class Camera:
    def __init__(self) -> None:
        pics_path = Path('cat pics/')
        self._pictures = list(pics_path.glob('*'))
        self._iterator = 0

    @property
    def frame(self):
        rframe = cv2.imread(str(self._pictures[self._iterator].resolve()))
        self._iterator = (self._iterator + 1) % len(self._pictures)
        return rframe