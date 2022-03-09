# Robotics
A repository for our assignments in Autonomous Mobile Robotics.

## How to run
**1)** Clone repository into the `src/` folder of your catkin workspace.
```bash
git clone https://github.com/NicklasVraa/Robotics
```
**2)** Build your workspace from the root workspace folder.
```bash
catkin build
```
**3)** Re-source the `devel/` folder.
```bash
source ~/your_catkin_ws/devel/setup.bash
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

## How to edit
**1)** Edit your local copy however you would like. \
**2)** Add all changes locally.
```bash
git add -A
```
**3)** Commit added changes locally.
```bash
git commit -m "Some message"
```
**4)** Push local changes to the online main branch.
```bash
git push -u origin main
```

Authors: N. Vraa, D. Felsager, A. Noertoft
