# Ackermann Line Follower Robot

This is a simulation of a line follower robot that works with steering control based on [Stanley: The Robot That Won the DARPA Grand Challenge](http://robots.stanford.edu/papers/thrun.stanley05.html) and computer vision techniques. 

The steering control is applied to a vehicle that has an embedded camera and uses Ackermann steering mechanism. The simulation was made using the [CoppeliaSim simulator](https://www.coppeliarobotics.com/) EDU version 4.0.0.

![Alt text](https://github.com/lucasmazz/ackermann-line-follower-robot/blob/master/ackermann-line-follower-robot.png?raw=true)

## Requirements

- CoppeliaSim
- Python 3.8 
- OpenCV

Install Python OpenCV:

```
pip install python-opencv
```

## How to Run

To run this simulation, download CoppeliaSim and clone this repository. Copy the CoppeliaSim remoteApi library file present in CoppeliaSim files to the simulation directory in this repository. This file is usually found inside the `CoppeliaSim/programming/remoteApiBindings/lib/lib` folder. 

The library file is different for each operating system, so you will need to copy different files if you are on different systems. If you are using Linux, copy the `remoteApi.so` file. But, if you are using Windows or MacOS, copy the `remoteApi.dll` file or copy `remoteApi.dylib` file, respectively.

For instance, with Ubuntu 18 Linux: 

```
git clone https://github.com/lucasmazz/ackermann-line-follower-robot.git
cp CoppeliaSim/programming/remoteApiBindings/lib/lib/Ubuntu18_04/remoteApi.so ackermann-line-follower-robot/simulator
```

Then, open the simulation scene `simulation.ttt` present in the root folder of this repository with CoppeliaSim simulator. Next, run the `run.py` script also present in the root folder of this repository, as follows:
 
```
cd ackermann-line-follower-robot
python run.py --debug --speed 20 --k 0.2
```

Enjoy.

