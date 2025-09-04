# Setup Commands: (inside VSCode terminal)
## (one-time) python -m venv .venv
## (Windows: every re-open) ./.venv/Scripts/activate.bat
## (Other systems: every re-open) ./.venv/Scripts/activate
## (one-time) pip install matplotlib opencv-python numpy
import numpy as np
import matplotlib.pyplot as plt
import cv2

import math
pi = math.pi
e = math.e

def convolve(image, kernel):

    """
    Code Applies a convolution to an image using a given kernel.

    Code handles different kernel sizes

    Tests:
        kernel must have odd rows and cols
        kernel & image must be a numpy array
        kernel must have one channel
    """
    # 2na ma3raf4 leih karart 23mel le colours wa 2a3azeb nafsy :/ kont fakerha sahla

    if not isinstance(kernel, np.ndarray) or not isinstance(image, np.ndarray):
        raise TypeError(f"{kernel} and {image} must be a numpy array")
    if not len(kernel.shape) == 2:
        raise TypeError(f"{kernel} must have only one channel")

    # if grayscale then add channel dimension
    if len(image.shape) == 2:
        image = image[:, :, np.newaxis]

    image_n, image_m, image_channels = image.shape
    kernel_n, kernel_m = kernel.shape               

    if kernel_n%2 != 1 or kernel_m%2 != 1:
        print(f"kernel rows = {kernel_n}"); print(f"kernel cols = {kernel_m}")
        raise TypeError("Kernel must have odd rows and columns")

    kernel = kernel[:, :, np.newaxis]  # adding a dimension dor np.sum to work
    kernel = kernel[:, ::-1, :] # flipping kernel horizontally
    kernel = kernel[::-1, :, :] # flipping kernel vertically

    # padding:
    pad_n = kernel_n // 2; pad_m = kernel_m // 2
    padded_image = np.pad(image, ((pad_n, pad_n), (pad_m, pad_m), (0, 0)),
                           mode='constant', constant_values=0)


    result_n = image_n
    result_m = image_m

    if image_channels == 1:
        result = np.zeros((result_n, result_m))
    else:
        result = np.zeros((result_n, result_m, image_channels))


    for i in range(result_n):
        for j in range(result_m):
            image_part = padded_image[i : kernel_n + i, j : kernel_m + j, :]
            result_pixel = np.sum(image_part * kernel, axis=(0, 1)) #axis 2 (colours) not included; hence, return one value for each channel/colour
            # like-> for all k: result_pixel = np.sum(image_part[k] * kernel[k]) // done for k=1, k=2, ...
            # output is a vector where each index represent a colour/channel

            if image_channels == 1:
                result[i, j] = result_pixel[0] # vector consist of one element -> na5od 2l value beta3oh b2a
            else:
                result[i, j, :] = result_pixel # ne7ot 2l vector values kol wa7ed le its colour

    if len(image.shape) == 2: #remove extra dimension if it was greyscale
        result = result[:, :, 0]

    return result

def guassian_calc(i, j, sigma):
    i_squared = i * i
    j_squared = j * j
    sigma_squared = sigma * sigma
    res = (1 / (2 * pi * sigma_squared)) * (e ** -((i_squared + j_squared) / (2 * sigma_squared)))
    return res

def guassian_kernal(height, width, sigma):
    res = np.zeros((height, width))
    for i in range(height):
        for j in range(width):
            res[i][j] = guassian_calc(i, j, sigma)
    return res

# Take notice that OpenCV handles the image as a numpy array when opening it
img1 = cv2.imread('Session 3/for_sobel.png', cv2.IMREAD_GRAYSCALE)

fig, axes = plt.subplots(3, 2, figsize=(8, 8))

axes[0, 0].imshow(img1, cmap='gray')
axes[0, 0].set_title('Original Image 1')
axes[0, 0].axis('off')

axes[0, 1].imshow(convolve(img1, np.ones((5, 5)) / 25), cmap='gray')
axes[0, 1].set_title('Box Filter')
axes[0, 1].axis('off')

axes[1, 0].imshow(convolve(img1, np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])), cmap='gray')
axes[1, 0].set_title('Horizontal img1 Filter')
axes[1, 0].axis('off')

axes[1, 1].imshow(convolve(img1, np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])), cmap='gray')
axes[1, 1].set_title('Vertical img1 Filter')
axes[1, 1].axis('off')

axes[2, 0].imshow(convolve(img1, guassian_kernal(3, 3, sigma = 7)), cmap='gray')
axes[2, 0].set_title('Gaussian filter')
axes[2, 0].axis('off')

plt.show()