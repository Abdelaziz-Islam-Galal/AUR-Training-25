import numpy as np
import matplotlib.pyplot as plt
import cv2
# from enum import Enum -> tried but failed and changed strategy

# Take notice that OpenCV handles the img as a numpy array when opening it 
img = cv2.imread('Session 3/shapes.jpg')
out = img.copy()  # type: ignore

BLUE_MASK = (img[:, :, 0] >= 150) & (img[:, :, 1] <= 50) & (img[:, :, 2] <= 50) # type: ignore
RED_MASK = (img[:, :, 0] <= 50) & (img[:, :, 1] <= 50) & (img[:, :, 2] >= 150) # type: ignore
BLACK_MASK = (img[:, :, 0] <= 50) & (img[:, :, 1] <= 50) & (img[:, :, 2] <= 50) # type: ignore


out[BLUE_MASK] = [0, 0, 0]
out[RED_MASK] = [255, 0, 0]
out[BLACK_MASK] = [0, 0, 255]

# Change all pixels that fit within the blue mask to black
# Change all pixels that fit within the red mask to blue
# Change all pixels that fit within the black mask to red

# BGR to RGB for printing
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
out = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

fig, axes = plt.subplots(1, 2)
axes[0].imshow(img)
axes[0].set_title('Original img')
axes[0].axis('off')

axes[1].imshow(out)
axes[1].set_title('Processed img')
axes[1].axis('off')

plt.show()