import numpy as np
import matplotlib.pyplot as plt
import cv2
# Take notice that OpenCV handles the image as a numpy array when opening it 
img = cv2.imread('shapes.jpg')
out = img.copy()


# Make a mask for each color (red, blue, black)
# Take care that the default colorspace that OpenCV opens an image in is BGR not RGB

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