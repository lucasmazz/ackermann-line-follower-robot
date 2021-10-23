import sys
import numpy as np
from simulator import sim


class Camera:
    def __init__(self, connection_id, name):
        """Connects to a camera in simulation.

        Args:
            connection_id (int): connection id
            name (str): sensor name
        """
        self.connection_id = connection_id

        # Init cam handler.
        error_code, self.handler = sim.simxGetObjectHandle(
            self.connection_id, name, sim.simx_opmode_blocking)

        # Init sensors.
        error_code, _, _ = sim.simxGetVisionSensorImage(
            self.connection_id, self.handler, 0, sim.simx_opmode_streaming)

    @property
    def image(self):
        """Get the current image from the camera.

        Returns:
            numpy.ndarray: resized image
        """
        error_code, resolution, image = sim.simxGetVisionSensorImage(
            self.connection_id, self.handler, 0, sim.simx_opmode_buffer)

        # Failed to get image.
        if error_code == 1:
            return None

        image = np.array(image, dtype=np.uint8)
        image.resize([resolution[0], resolution[1], 3])
        return image
