import cv2
from threading import Thread
# from pathlib import Path
# import numpy as np

CAPTURE_DEVICE = 0

class Camera:
    def __init__(self) -> None:
        # Original Code (to display cat pics):
        
        # pics_path = Path('cat pics/')
        # self._pictures = list(pics_path.glob('*'))
        # self._iterator = 0

        # Task 1 Code:

        self._capture_device = cv2.VideoCapture()
        self._capture_device.open(CAPTURE_DEVICE)

    @property
    def frame(self):
        # Original Code (to display cat pics):
        
        # rframe = cv2.imread(str(self._pictures[self._iterator].resolve()))
        # self._iterator = (self._iterator + 1) % len(self._pictures)
        # return rframe
        
        # Task 1 Code:

        capture = self._capture_device.read()
        # capture is a typle (bool isSuccess, frame array)
        if capture[0]:
            return capture[1]
        else:
            print("an error in capturing a frame occured!")
            
            # Close and Open device agian because the problem may be in the camera itself
            self._capture_device.release()
            self._capture_device.open(CAPTURE_DEVICE)

