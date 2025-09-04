import numpy as np
import matplotlib.pyplot as plt
import cv2
from enum import Enum
# Take notice that OpenCV handles the image as a numpy array when opening it 
img = cv2.imread('shapes.jpg')
out = img.copy()

class Red(Enum):
    BLUE_LOW = 0
    BLUE_HIGH = 50
    GREEN_LOW = 0
    GREEN_HIGH = 50
    RED_LOW = 150
    RED_HIGH = 255
class Blue(Enum):
    BLUE_LOW = 150
    BLUE_HIGH = 255
    GREEN_LOW = 0
    GREEN_HIGH = 50
    RED_LOW = 0
    RED_HIGH = 50
class Black(Enum):
    BLUE_LOW = 0
    BLUE_HIGH = 50
    GREEN_LOW = 0
    GREEN_HIGH = 50
    RED_LOW = 0
    RED_HIGH = 50

# Change all pixels that fit within the blue mask to black
# Change all pixels that fit within the red mask to blue
# Change all pixels that fit within the black mask to red

fig, axes = plt.subplots(1, 2)
axes[0].imshow(img)
axes[0].set_title('Original Image')
axes[0].axis('off')

axes[1].imshow(out)
axes[1].set_title('Processed Image')
axes[1].axis('off')

plt.show()