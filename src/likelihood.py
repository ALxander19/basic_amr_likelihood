#!/usr/bin/env python
import rospy
import tf
import numpy as np
from scipy.stats import norm
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

global x_prime_list, y_prime_list
x_prime_list = y_prime_list = 0

def laser_callback(msg_laser):
  
  act_pose = Pose()
  act_pose.position.x = -2.0
  act_pose.position.y = -0.5
  act_pose.orientation.w = 1
  
  print(likelihood(msg_laser,act_pose,x_prime_list,y_prime_list))

def map_callback(msg_map):
  global x_prime_list, y_prime_list
  load_map = msg_map
  data_array = np.asarray(msg_map.data)
  data_array = np.reshape(data_array, (msg_map.info.width, msg_map.info.height))
  x_prime_list = []
  y_prime_list = []

  for i in range(1,msg_map.info.width): 
    for j in range(1,msg_map.info.height):
      if data_array[i][j]==100:
        x_prime_list.append(i*msg_map.info.resolution+msg_map.info.origin.position.x)
        y_prime_list.append(j*msg_map.info.resolution+msg_map.info.origin.position.y)

def likelihood(msg_laser,act_pose,x_prime_list,y_prime_list):

  z_max = msg_laser.range_max
  z_hit = 0.95; z_random = 1 - z_hit; variance_hit = 0.04
  thetak_sens = msg_laser.angle_min
  q = 1
  count = 0
  for z_k in msg_laser.ranges:


  print("cuenta: "+str(count))
  return q**(1./count)


if __name__ == "__main__":

  rospy.init_node('likelihood_field')
  rospy.Subscriber('scan', LaserScan, laser_callback)
  rospy.Subscriber('map', OccupancyGrid, map_callback)

  rospy.spin()
  