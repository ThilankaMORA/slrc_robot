#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

I=0
previousError=0
PIDvalue=0

speed = 3

Kp,Ki,Kd=map(float,input().strip().split())

def callback(data):
  ir_list = list(map(int,list(("0" * (8-len(bin(data.data)[2:])))+bin(data.data)[2:])))
  print(ir_list)
  error= (-8)*ir_list[0]+(-4)*ir_list[1]+(-2)*ir_list[2]+(-1)*ir_list[3]+1*ir_list[4]+2*ir_list[5]+4*ir_list[6]+8*ir_list[7]
  calculatePID(error)

def calculatePID(error):
  P = error
  global I,previousError
  I = I + error
  D = error-previousError
  global PIDvalue
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D)
  previousError = error

def talker():
  pub = rospy.Publisher('cmd_vel', String, queue_size=10)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    speedpub = str(speed-PIDvalue)+","+str(speed+PIDvalue)
    pub.publish(speedpub)
    rate.sleep()

def listener():
  
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("ir_array_values", UInt8, callback)
  talker()
  rospy.spin()
   
if __name__ == '__main__':
  listener()


 