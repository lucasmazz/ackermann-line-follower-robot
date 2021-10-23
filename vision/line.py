import cv2
import numpy as np


def compute_centroids(image, n=8):
    """Computes the line centroids, given a segmented image.

    Args:
        image (numpy.ndarray): segmented image
        n (int, optional): number of centroids

    Returns:
        tuple: detected centroids and weights of each centroid
    """
    w = image.shape[1]
    h = image.shape[0]

    segment_size = w/n

    centroids = np.zeros([n, 2])
    weights = np.zeros(n)

    # Split the image in n different segments
    # and locate the centroid of each segment.
    for i in range(n):
        seg_min = int(i*segment_size)
        seg_max = int(i*segment_size + segment_size)

        segment = image[seg_min:seg_max, :]

        # Sum all rows of each segment.
        row_projection = np.sum(segment, axis=0)

        if not row_projection.any():
            continue

        cX = np.dot(row_projection, np.arange(row_projection.shape[0]) /
             np.sum(row_projection))

        cY = seg_max

        centroids[i, 0] = cX
        centroids[i, 1] = cY
        weights[i] = np.sum(row_projection[:])

    return centroids, weights
