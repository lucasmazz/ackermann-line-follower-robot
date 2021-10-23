import sys
from math import pi, sqrt, degrees, radians, tan, atan, sin, cos
from simulator import sim


class Car:
    def __init__(self, connection_id,
                 motor_left, motor_right, steering_left, steering_right,
                 d=0.755, l=2.5772):
        """Handles the simulated car.

        Args:
            connection_id (int): connection id
            motor_left (str): left motor name
            motor_right (str): right motor name
            steering_left (str): left steering name
            steering_right (str): right steering name
            d (float): distance between left and right wheels
            l (float): distance between front and read wheels
        """
        self.connection_id = connection_id
        self.d = d
        self.l = l

        self.__speed_level = 0
        self.__steering_angle = 0

        error_code, self.motor_left_handler = sim.simxGetObjectHandle(
            connection_id,
            motor_left,
            sim.simx_opmode_blocking
        )

        assert(error_code == 0)

        error_code, self.motor_right_handler = sim.simxGetObjectHandle(
            connection_id,
            motor_right,
            sim.simx_opmode_blocking
        )

        assert(error_code == 0)

        error_code, self.steering_left_handler = sim.simxGetObjectHandle(
            connection_id,
            steering_left,
            sim.simx_opmode_blocking
        )

        assert(error_code == 0)

        error_code, self.steering_right_handler = sim.simxGetObjectHandle(
            connection_id,
            steering_right,
            sim.simx_opmode_blocking
        )

        assert(error_code == 0)

    def __ackermann_steering(self, angle):
        """Gives the steering angles of ackermann steering.

        Args:
            angle (float): desired steering angle

        Returns:
            tuple: steering angles from the left and right wheel   
        """
        steering_left = 0
        steering_right = 0

        if (angle > 0):
            steering_left = atan((2*self.l*sin(angle)) /
                                  (2*self.l*cos(angle) - self.d*sin(angle)))

            steering_right = atan((2*self.l*sin(angle)) /
                                 (2*self.l*cos(angle) + self.d*sin(angle)))
        elif (angle < 0):
            angle = angle*-1

            steering_left = -atan((2*self.l*sin(angle)) /
                                   (2*self.l*cos(angle) + self.d*sin(angle)))

            steering_right = -atan((2*self.l*sin(angle)) /
                                  (2*self.l*cos(angle) - self.d*sin(angle)))

        return (steering_left, steering_right)

    @property
    def steering_angle(self):
        return self.__steering_angle

    @steering_angle.setter
    def steering_angle(self, angle):
        self.__steering_angle = angle

        steering_left, steering_right = self.__ackermann_steering(angle)

        sim.simxSetJointTargetPosition(self.connection_id, self.steering_left_handler,
                                       steering_left, sim.simx_opmode_streaming)

        sim.simxSetJointTargetPosition(self.connection_id, self.steering_right_handler,
                                       steering_right, sim.simx_opmode_streaming)

    @property
    def speed_level(self):
        return self.__speed_level

    @speed_level.setter
    def speed_level(self, speed):
        self.__speed_level = speed

        sim.simxSetJointTargetVelocity(
            self.connection_id, self.motor_left_handler, self.__speed_level, sim.simx_opmode_streaming)

        sim.simxSetJointTargetVelocity(
            self.connection_id, self.motor_right_handler, self.__speed_level, sim.simx_opmode_streaming)
