import sys
from . import sim


class SimConnection:

    def __init__(self, addr='127.0.0.1', port=19999):
        self.close() # just in case, close all opened connections
        self.id = sim.simxStart(
            addr, port, True, True, 5000, 5)

        if self.id != -1:
            print ("Connected to remote API server")
        else:
            print("Not connected to remote API server")

    def close(self):
        sim.simxFinish(-1) 



