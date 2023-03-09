#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import tf

def get_yaw(data):
  x = data.pose.pose.orientation.x
  y = data.pose.pose.orientation.y
  z = data.pose.pose.orientation.z
  w = data.pose.pose.orientation.w

  q=[x,y,z,w]

  rpy = tf.transformations.euler_from_quaternion(q)
  global yaw
  yaw = math.degrees(rpy[2])

def turn(data):
  if (data.data == "left") : turn_left(yaw+90)
  elif (data.data == "right") : turn_right(yaw-90)
  elif (data.data == "back") : turn_back(yaw+180)

def turn_left():
def turn_right():
def turn_back():

def listener():
    
  rospy.init_node('listener', anonymous=True)
   
  rospy.Subscriber("/odom", Odometry, get_yaw)
  rospy.Subscriber("/cmd_dir",String,turn) 
  rospy.spin()
   
if __name__ == '__main__':
  listener()