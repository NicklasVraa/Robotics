#!/usr/bin/env python

""" Desc: A run script for homework 1 in Autonomous Mobile Robots.
    Auth: Nicklas Vraa, David Felager, Andreas Joergensen. """

import rospy
import matplotlib.pyplot as plt
from robotics import Controller # Import our own library.
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time

def get_input(prompt):
  """ Get input from user, while pausing simulation. """
  pause()
  ans = input(prompt)
  unpause()
  return ans

def reset_state():
  """ Resets model states. """
  msg = ModelState()
  msg.model_name = 'husky'
  msg.pose.position.x = 0
  msg.pose.position.y = 0
  msg.pose.position.z = 0.18
  msg.pose.orientation.x = 0
  msg.pose.orientation.y = 0
  msg.pose.orientation.z = 0
  msg.pose.orientation.w = 0

  reset_bot(msg)

if __name__ == '__main__':
  """ Main function. Use: rosrun husky_controllers tasks.py """

  try:
    # Setup of plotting and Gazebo control from within this script.
    plt.rcParams.update({'font.size': 12})

    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/set_model_state')

    reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    reset_bot = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Start of actual program. Prompt user for input.
    task = str(get_input('What task should be run? (1.1, 1.2 ... or 2): '))
    robot = Controller([1.5, 3, -0.1], True)

    if task == '1.1':
      x = get_input('Enter x: ')
      y = get_input('Enter y: ')
      robot.move_to([x, y, 1], 0.1)
      robot.plot_trajectory('Moving to point')
      robot.plot_rho()

    elif task == '1.2':
      a = get_input('Enter a-coefficient: ')
      robot.follow_line(a, 5, 0.1)
      robot.plot_trajectory('Moving through y=ax')
      robot.plot_rho()

    elif task == '1.3':
      a = get_input('Enter a-coefficient: ')
      robot.follow_curve(a, 10, 0.1)
      robot.plot_trajectory('Moving through y=ax^2')

    elif task == '1.4':
      path = [[2, 2], [0, 4]]

      rho_params   = [[0.5, 3, 0], # Rho < 1 is too slow.
                      [1.0, 3, 0], # Rho > 2 has bigger swing.
                      [2.0, 3, 0], # Rho > 3 does not increase speed.
                      [4.0, 3, 0],
                      [8.0, 3, 0]]

      alpha_params = [[8.0, 1.0, 0], # Alpha < 1 will not converge.
                      [8.0, 2.0, 0], # Alpha > 4 makes backend slide.
                      [8.0, 4.0, 0],
                      [8.0, 8.0, 0]]

      beta_params  = [[8.0, 3.0,  1.5], # Beta should stay around 0.
                      [8.0, 3.0,    0],
                      [8.0, 3.0, -0.1],
                      [8.0, 3.0,   -2]]

      ideal_params =  [[1.5, 3, -0.1]] # Both good speed and accuracy.

      for param in rho_params: # Change in code.
        pause()
        reset_state()
        robot.reset()
        robot.set_params(param)
        unpause()
        time.sleep(1)
        print('Using ' + str(param) + ':')
        robot.move_through(path, 0.15)
        robot.plot_trajectory('Tuning', False, str(param))
        time.sleep(1)

      plt.show()

    elif task == '2':
      robot.ns = get_input('Enter namespace value (1=empty, 2=maze): ')
      path = [[11.5, -0.5],
              [11.5,  5.2],
              [4.0,   5.0],
              [2.3,  -0.5]]

      robot.move_through(path, 0.1)
      robot.plot_trajectory_2(path)
      robot.plot_est_gnd_dist()
      robot.plot_est_gnd_yaw()

    else:
      print('Not a valid task.')
      print('Options: 1.1, 1.2, 1.3, 1.4 or 2')

    reset_sim()

  except rospy.ROSInterruptException:
    pass
