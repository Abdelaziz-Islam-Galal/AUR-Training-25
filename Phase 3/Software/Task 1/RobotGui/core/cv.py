import cv2
from threading import Thread
from time import sleep
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

        # Session 2 continouing the code:
        self._empty_frame = cv2.imread('cat pics/felfel.jpg')
        self._frame = None
        self._frame_thread = Thread(target = self._frame_loop, daemon = True) # daemon = True so when window is closed it does not wait for this thread to finish
        self._frame_thread.start()
        # Note:
        # The thread asks for a frame before the device connects and sends its frame; hence, we used an empty_frame to fill in this time

    @property
    def frame(self):
        # Original Code (to display cat pics):
        
        # rframe = cv2.imread(str(self._pictures[self._iterator].resolve()))
        # self._iterator = (self._iterator + 1) % len(self._pictures)
        # return rframe
        
        # Task 1 Code:

        # capture = self._capture_device.read()
        # # capture is a typle (bool isSuccess, frame array)
        # if capture[0]:
        #     return capture[1]
        # else:
        #     print("an error in capturing a frame occured!")
            
        #     # Close and Open device agian because the problem may be in the camera itself
        #     self._capture_device.release()
        #     self._capture_device.open(CAPTURE_DEVICE)

        # Session 2 Code: -> applied threading so capturing images does not slow down the gui in the main thread

        return self._frame if self._frame is not None else self._empty_frame

    # for Session 2 Code
    def _frame_loop(self):
        while True:
            success, image = self._capture_device.read()
            # capture is a typle (bool isSuccess, frame array)
            if success:
                self._frame = image
            else:
                print("an error in capturing a frame occured!")
                # Close and Open device agian because the problem may be in the camera itself
                self._capture_device.release()
                self._capture_device.open(CAPTURE_DEVICE)

            sleep(0.015) # 15 ms delay so this function is a bit slower so it does not consume most of the cpu power
            # we chose 15 ms while the frames update every 50 ms (we chose that value in function update)
            # so if a problem occurs it can be detected before the next frame is sent
