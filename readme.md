# Robotics
A repository for our assignments in Autonomous Mobile Robotics.

## How to run
**1)** Clone repository in the `src/` folder of your catkin workspace.
```bash
git clone https://github.com/NicklasVraa/Robotics
```
**2)** Build your workspace.
```bash
catkin build
```
**3)** Re-source the `devel/` folder.
```bash
source ~/ros_ws/devel/setup.bash
```
**4)** Launch gazebo simulation.
```bash
roslaunch husky_gazebo husky_empty_world.launch
# or
roslaunch Robotics maze.launch
```
**5)** Run our script.
```bash
rosrun Robotics tasks.py
```

Authors: N. Vraa, D. Felsager, A. Noertoft
