import math
import sys
import time
import argparse
from math import degrees, radians

from simulator import SimConnection
from control import steering_control
from sensors import Camera
from drivers import Car
from vision import *


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true',
                        help='enable debug')
    
    parser.add_argument('-s', '--speed', type=float, default=20, 
                        help='car speed level')
    
    parser.add_argument('-k', '--k', type=float, default=0.25, 
                        help='control constant')
    
    args = parser.parse_args()
    
    conn = SimConnection()

    if conn.id == -1:
        sys.exit("Could not connect.")

    # Communication with the simulation car.
    car = Car(
        conn.id,
        motor_left="nakedCar_motorLeft",
        motor_right="nakedCar_motorRight",
        steering_left="nakedCar_steeringLeft",
        steering_right="nakedCar_steeringRight"
    )

    # Communication with the car's front camera.
    cam = Camera(conn.id, name="Vision_sensor")

    # Set a constant speed.
    car.speed_level = args.speed

    while True:
        # Get the current image from camera.
        image = cam.image

        if image is not None:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 0)
            line = line_segmentation(image)

            # Uses the segmented line to compute lines centroids with weights.
            centroids, weights = compute_centroids(line, 8)

            # Gives the steering angle.
            theta = steering_control(centroids, weights, args.speed, image, 
                                     k=args.k, debug=args.debug)
            car.steering_angle = theta

            if args.debug:
                cv2.imshow('image', image)
                cv2.imshow('segmentation', line)
                cv2.waitKey(1)
