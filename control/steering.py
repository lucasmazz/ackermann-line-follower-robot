import cv2
import numpy as np
from math import radians


__centroids = []


def filter_centroids(centroids, weights, image, debug):
    global __centroids
    
    if len(__centroids) != 0:

        for i, pt in enumerate(centroids):

            # If the weight is not too small, then stores the centroid.
            if weights[i] > 10:
                __centroids[i] = centroids[i]

                if debug:
                    cv2.circle(image, (int(pt[0]), int(pt[1])),
                               5, (0, 255, 0), -1)

            # If the weights is less than 10, then use the stored centroid.
            elif debug:
                cv2.circle(image, (int(pt[0]), int(pt[1])),
                           5, (0, 0, 255), -1)

                cv2.circle(image, (int(__centroids[i][0]),
                                   int(__centroids[i][1])),
                           5, (0, 255, 255), -1)
    else:
        __centroids = centroids

    return __centroids


def compute_crosstrack_error(centroids, image, debug):
    # Computes the error between a centroid and the center of image.
    errors = np.zeros(centroids.shape[0])

    for i in range(centroids.shape[0]):
        p = centroids[i, :]
        errors[i] = p[0] - (image.shape[1]/2)

    if debug:
        cv2.line(image, (int(p[0]), int(p[1])),
                 (int(image.shape[0]/2), int(image.shape[1]/2)),
                 (0, 0, 255), 5)

    # Get last error only.
    return errors[-1]


def compute_angular_error(centroids, tail, head, image, debug):
    # Computes the line vector and normalize it.
    v = (tail - head)
    v = v/np.linalg.norm(v)

    # Computes the angle between this line and the central image line.
    angle_error = np.arcsin(v[0])

    if debug:
        cv2.line(image, (int(tail[0]), int(tail[1])),
                 (int(head[0]), int(head[1])),
                 (255, 0, 0), 5)

    return angle_error


def steering_control(centroids, weights, v, image, k=0.33, debug=False):
    """Steering control based on Stanley robot control.

    Args:
        centroids (list): line centroids
        weights (list): centroids weights
        v (float): car speed
        image (numpy.ndarray): camera image
        k (float): cross track error gain
        debug (bool): actives the debug mode. Defaults to False

    Returns:
        float: steering angle in radians
    """
    # Removes the low weights centroids.
    c = filter_centroids(centroids, weights, image, debug)

    # Defines the tail and head of the vector used to compute angular error.
    tail = c[-1]
    head = c[int(len(c)/2)-1]

    angular_error = compute_angular_error(c, tail, head, image, debug)
    cross_error = compute_crosstrack_error(c, image, debug)
    
    # Steering angle control based on the Stanley robot.
    total_error = angular_error - np.arctan2(k*cross_error, v)

    return total_error
