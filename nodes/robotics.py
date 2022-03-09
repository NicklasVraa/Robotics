#!/usr/bin/env python

""" Desc: A custom library for working with robots.
    Auth: Nicklas Vraa, David Felager, Andreas Joergensen. """

import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sqrt, pow, sin, cos
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# Local utility functions.
def quat2yaw(q):
  """ A simplified quaterion to euler for 2D space. """
  return atan2(+2.0*(q.w*q.z + q.x*q.y), +1.0 - 2.0*(q.y*q.y + q.z*q.z))

def norm_angle(theta):
  """ Normalize the given angle. """
  return atan2(sin(theta), cos(theta))

# Public classes.
class Controller:
  """ A generic class for any ground robot. """

  def __init__(self, param, logging):
    """ Constructor. """

    # ROS setup.
    rospy.init_node('Ground_robot', anonymous=True)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.sub_1 = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_state)
    self.sub_2 = rospy.Subscriber('/odometry/filtered', Odometry, self.estimate_state)
    self.rate = rospy.Rate(100)
    self.ns = 1 # Namespace value in gazebo.

    self.sample_freq = 5.0 # Hz
    self.prev_sample_time = rospy.get_time()
    self.dt = float(1/self.sample_freq)
    self.logging = logging

    self.set_params(param)
    self.reset()

  def set_params(self, param):
    self.K_rho = param[0]
    self.K_alpha = param[1]
    self.K_beta = param[2]

  def reset(self):
    """ Set/reset states, errors and logs. """
    # States and parameters.
    self.x = self.y = self.yaw = 0         # Current state.
    self.x_p = self.y_p = self.yaw_p = 0   # Previous target state.
    self.x_t = self.y_t = self.yaw_t = 0   # Current target state.
    self.rho = self.alpha = self.beta = 0  # Errors.
    self.tolerance = 1

    # Ground truth and estimates
    self.dists_travelled = [0]; self.dist_travelled = 0
    self.est_dists_travelled = [0]; self.est_dist_travelled = 0
    self.est_yaws = [0]; self.est_yaw = 0
    self.linear_velocities = [0]
    self.angular_velocities = [0]

    # For plotting.
    self.ts = [0]; self.xs = [0]; self.ys = [0]; self.yaws = [0]  # State logs.
    self.rhos = []; self.alphas = []; self.betas = []             # Error logs.


  def update_state(self, data):
    """ Updates the states of the robot. """
    self.x = data.pose[self.ns].position.x
    self.y = data.pose[self.ns].position.y
    self.yaw = quat2yaw(data.pose[self.ns].orientation)
    self.yaw = norm_angle(self.yaw)

    if self.logging:
      self.log()

  def estimate_state(self, data):
    """ Estimates the distance travelled and yaw angle of the ground robot. """
    self.linear_velocities.append(np.sqrt(data.twist.twist.linear.x**2+data.twist.twist.linear.y**2))
    self.angular_velocities.append(data.twist.twist.angular.z)
    self.est_dist_travelled += self.dt*(self.linear_velocities[-2] + self.linear_velocities[-1])/2
    self.est_yaw += self.dt*(self.angular_velocities[-2] + self.angular_velocities[-1])/2
    self.est_yaw = norm_angle(self.est_yaw)

  def log(self):
    """ Log states at an appropriate rate. """
    now = rospy.get_time()
    self.dt = now - self.prev_sample_time
    if self.dt >= 1.0 / self.sample_freq:
      self.ts.append(self.ts[-1] + self.dt)
      self.xs.append(self.x)
      self.ys.append(self.y)
      self.dist_travelled += sqrt((self.xs[-2]-self.xs[-1])**2+(self.ys[-2]-self.ys[-1])**2)
      self.dists_travelled.append(self.dist_travelled)
      self.est_dists_travelled.append(self.est_dist_travelled)
      self.yaws.append(self.yaw)
      self.est_yaws.append(self.est_yaw)
      self.rhos.append(self.rho)
      self.prev_sample_time = now

  def calc_rho(self):
    """ Returns positional error. """
    self.rho = sqrt(pow(self.x - self.x_t, 2) + pow(self.y - self.y_t, 2))
    return self.rho

  def calc_alpha(self):
    """ Returns steering angle error. """
    self.alpha = -self.yaw + atan2(self.y_t - self.y, self.x_t - self.x)
    self.alpha = norm_angle(self.alpha)
    return self.alpha

  def calc_beta(self):
    """ Returns orientational error. """
    self.beta = - (self.yaw + self.calc_alpha()) + self.calc_offset()
    self.beta = norm_angle(self.beta)
    return self.beta

  def calc_offset(self):
    return atan2(self.y_t - self.y_p, self.x_t - self.x_p)

  def move_to(self, goal, tolerance, const_speed=False):
    """ Move the robot to a given coordinate. """
    self.tolerance = tolerance
    self.x_t = goal[0]; self.y_t = goal[1]; self.yaw_t = goal[2]

    msg = Twist()
    print('Moving to ' + str(self.x_t) + ',' + str(self.y_t) + '...')

    while self.calc_rho() >= tolerance:
      if const_speed:
        msg.linear.x = self.K_rho * 1
      else:
        msg.linear.x = self.K_rho * self.calc_rho()

      msg.angular.z = self.K_alpha * self.calc_alpha() + self.K_beta * self.calc_beta()
      self.pub.publish(msg)
      self.rate.sleep()

    print('Reached destination.')
    self.stop()

  def stop(self):
    """ Set all velocities to zero and record goal. """
    msg = Twist()
    msg.linear.x = msg.angular.z = 0
    self.pub.publish(msg)

    self.x_p = self.x_t
    self.y_p = self.y_t
    self.yaw_p = self.yaw_t

  def plot_trajectory(self, title, show=True, label='Trajectory'):
    """ Plot robot trajectory. """
    theta = np.linspace(0, 2*np.pi, 200)
    plt.plot(np.ones(200)*self.x_t + self.tolerance*np.cos(theta),
             np.ones(200)*self.y_t + self.tolerance*np.sin(theta),
            ':', linewidth=2, color='black', label='Goal threshold')

    plt.plot(self.x_t, self.y_t, 'o', color='black', label='Goal')
    plt.plot(self.xs, self.ys, mfc='none', label=label)
    plt.title(title); plt.xlabel('x'); plt.ylabel('y'); plt.axis('equal')
    legend = plt.legend(loc='upper right'); plt.grid()

    if show:
      plt.show()

  def plot_trajectory_2(self, path):
    """ Plot robot trajectory. Specifically for task 2. """
    theta = np.linspace(0, 2*np.pi, 200)

    #Plot starting point.
    plt.plot(0, 0, 'o', mfc='none', color='blue', label='Start')

    # Plot waypoints.
    for point in path:
      x_t = point[0]; y_t = point[1]
      plt.plot(x_t, y_t, 'o', color='black', mfc='none',
              label='Waypoint: ' + str(point))

      plt.plot(np.ones(200)*x_t + self.tolerance*np.cos(theta),
                np.ones(200)*y_t + self.tolerance*np.sin(theta),
                ':', linewidth=2, color='black')

    # Plot goal point.
    plt.plot(self.x_t, self.y_t, 'o', color='red', mfc='none',
             label='Goal')
    plt.plot(np.ones(200)*x_t + self.tolerance*np.cos(theta),
             np.ones(200)*y_t + self.tolerance*np.sin(theta),
             ':', linewidth=2, color='black', label='Tolerance')

    plt.plot(self.xs[1:], self.ys[1:], color='red', mfc='none', label='Trajectory')
    plt.title('Path through maze'); plt.xlabel('x'); plt.ylabel('y'); plt.axis('equal')
    legend = plt.legend(loc='upper left'); plt.grid(); plt.show()

  def plot_rho(self):
    """ Plot robot positionaal error log. """
    plt.plot(self.ts[1:], self.rhos[0:]); plt.title('Distance to target')
    plt.xlabel('time [s]'); plt.ylabel('rho [m]')
    plt.grid(); plt.show()

  def plot_est_gnd_dist(self):
    """ Plot estimated distance travelled with ground truth. """
    plt.plot(self.ts, self.est_dists_travelled, label='Estimated distance travelled [m]')
    plt.plot(self.ts, self.dists_travelled, label='Ground truth distance travelled [m]')
    plt.title('Plots of estimated and ground truth: distance travelled [m]')
    plt.xlabel('t'); plt.ylabel('distance travelled [m]')
    legend = plt.legend(loc='upper left')
    plt.grid(); plt.show()

  def plot_est_gnd_yaw(self):
    """ Plot estimated yaw with ground truth. """
    plt.plot(self.ts, self.est_yaws, label='Estimated yaw [rad]')
    plt.plot(self.ts, self.yaws, label='Ground truth yaw [rad]')
    plt.title('Plots of estimated and ground truth: yaw [rad]')
    plt.xlabel('t'); plt.ylabel('yaw [rad]')
    legend = plt.legend(loc='upper right')
    plt.grid(); plt.show()

  def follow_line(self, a, length, tolerance):
    """ First order trajectory tracking. """
    x_t = length / sqrt(1 + a)
    self.move_to([x_t, a * x_t, a], tolerance)

  def move_through(self, path, tolerance, const_speed=False):
    """ Move through a set of points. """
    for point in path:
      self.move_to([point[0], point[1], 0], tolerance, const_speed)

  def follow_curve(self, a, resolution, tolerance):
    """ Second order trajectory tracking. """
    path = [[self.x, self.y]]
    self.K_rho = 1

    for i in range(resolution):
      x = path[i][0]
      path.append([x + 1 / sqrt(1 + 2*a*x), a*x**2])

    path.pop(0) # Remove current position from path.
    self.move_through(path, tolerance, True)
