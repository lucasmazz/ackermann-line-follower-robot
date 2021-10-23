import cv2
import numpy as np


def black_segmentation(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

    return th


def white_segmentation(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

    return th


def median_blur(image, kernel_size):
    blur = cv2.medianBlur(image, kernel_size)
    return blur


def line_segmentation(image):
    """Filter only the line borders in the image.

    Args:
        image (numpy.ndarray): complete BGR image 

    Returns:
        numpy.ndarray: filtered line
    """
    gray = white_segmentation(image) 
    blur = median_blur(gray, 5)

    return blur


