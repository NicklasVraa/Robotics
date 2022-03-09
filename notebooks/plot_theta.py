import rosbag
from bagpy import bagreader
import pandas as pd
import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
os.chdir('../bags')

bag = bagreader('/home/felsager/Workspaces/auro_ws/src/husky_controllers/bags/theta.bag')
pose_msg = bag.message_by_topic('gazebo_msgs/ModelStates')
print(bag.m)


