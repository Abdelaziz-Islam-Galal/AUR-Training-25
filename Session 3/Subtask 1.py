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

    return result

# ---------------------------------------------------------------

def gaussian_calc(i, j, sigma):
    i_squared = i * i
    j_squared = j * j
    sigma_squared = sigma * sigma
    res = (1 / (2 * pi * sigma_squared)) * (e ** -((i_squared + j_squared) / (2 * sigma_squared)))
    return res

def gaussian_kernal(height, width, sigma):
    res = np.zeros((height, width))
    center_i = height // 2
    center_j = width // 2
    for i in range(height):
        for j in range(width):
            dist_i = i - center_i
            dist_j = j - center_j
            res[i][j] = gaussian_calc(dist_i, dist_j, sigma)
    res = res / np.sum(res)
    return res

# ----------------------------------------------------------------

def median(list, size):
    if size%2 == 1:
        return list[size // 2]
    else:
        return (list[size // 2 - 1] + list[size // 2]) // 2

def median_filter(image, kernel_height, kernel_width):
    kernel_n = kernel_height; kernel_m = kernel_width

    if not isinstance(image, np.ndarray):
        raise TypeError(f"{image} must be a numpy array")
    
    # if grayscale then add channel dimension
    if len(image.shape) == 2:
        image = image[:, :, np.newaxis]

    image_n, image_m, image_channels = image.shape
    
    if kernel_n%2 != 1 or kernel_m%2 != 1:
        print(f"kernel rows = {kernel_n}"); print(f"kernel cols = {kernel_m}")
        raise TypeError("Kernel must have odd rows and columns")

    # padding:
    pad_n = kernel_n // 2; pad_m = kernel_m // 2
    padded_image = np.pad(image, ((pad_n, pad_n), (pad_m, pad_m), (0, 0)), mode='edge')
    # padding with zeros will cause problems whith median calculation
    # I dont know what to do but i do not want a smaller output
    # so i will replicate the borders, wa 2ma we 7azy b2a


    result_n = image_n
    result_m = image_m

    if image_channels == 1:
        result = np.zeros((result_n, result_m))
    else:
        result = np.zeros((result_n, result_m, image_channels))

    size = kernel_m * kernel_n
    for i in range(result_n):
        for j in range(result_m):
            image_part = padded_image[i : kernel_n + i, j : kernel_m + j, :]
            if image_channels == 1:
                kernel_list = image_part[:, :, 0].flatten().tolist()
                kernel_list.sort()
                result[i, j] = median(kernel_list, size)
            else:
                for k in range(image_channels):
                    kernel_list = image_part[:, :, k].flatten().tolist()
                    kernel_list.sort()
                    result[i, j, k] = median(kernel_list, size)

    return result

# ----------------------------------------------------------------

# most of the gui was made with the help of chatgpt
# I wanted to make more than one page and did not know how + had no time to read documentation
# I am just couple of hours away from the deadline and did not start subtask 2 yet :(
def create_comparison_page(img, title_suffix="", colour = None, box=True, horizontal=True, vertical=True, gaussian=True, median=True):
    fig, axes = plt.subplots(3, 2, figsize=(8, 8))
    
    axes[0, 0].imshow(img, cmap=colour)
    axes[0, 0].set_title(f'Original Image {title_suffix}')
    axes[0, 0].axis('off')

    if box:
        axes[0, 1].imshow(convolve(img, np.ones((5, 5)) / 25), cmap=colour)
        axes[0, 1].set_title('Box Filter')
        axes[0, 1].axis('off')
    if horizontal:
        axes[1, 0].imshow(convolve(img, np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])), cmap=colour)
        axes[1, 0].set_title('Horizontal Filter')
        axes[1, 0].axis('off')
    if vertical:
        axes[1, 1].imshow(convolve(img, np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])), cmap=colour)
        axes[1, 1].set_title('Vertical Filter')
        axes[1, 1].axis('off')
    if gaussian:
        axes[2, 0].imshow(convolve(img, gaussian_kernal(7, 7, sigma=1.5)), cmap=colour)
        axes[2, 0].set_title('Gaussian Filter')
        axes[2, 0].axis('off')
    if median:
        axes[2, 1].imshow(median_filter(img, 7, 7), cmap=colour)
        axes[2, 1].set_title('Median Filter')  # Fixed title
        axes[2, 1].axis('off')

    plt.tight_layout()
    return fig

grey_image_paths = [
    'Session 3/for_sobel.png',
]
bgr_image_paths = [
    'Session 3/shapes.jpg',
    # 'Session 3/for_median.jpg',
]

print("it is slow for coloured images idk why; just wait for it")

for i, img_path in enumerate(grey_image_paths):
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    fig = create_comparison_page(img, colour = 'gray')
    plt.show() 

for i, img_path in enumerate(bgr_image_paths):
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # type: ignore
    fig = create_comparison_page(img)
    plt.show()