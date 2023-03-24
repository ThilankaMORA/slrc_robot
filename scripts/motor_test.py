#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
  pub = rospy.Publisher('cmd_vel', String, queue_size=10)
  rospy.init_node('talker', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    speed = "5,5"
    pub.publish(speed)
    rate.sleep()
   
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass