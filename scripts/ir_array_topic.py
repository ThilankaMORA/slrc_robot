import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
import numpy as np

ir_array=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]) 

def callback_1(data):
  ir_array[0]=float(data.range)
  #print("ir_1",str(data.range))

def callback_2(data):
  ir_array[1]=float(data.range)
  #print("ir_2",str(data.range))

def callback_3(data):
  ir_array[2]=float(data.range)
  #print("ir_3",str(data.range))

def callback_4(data):
  ir_array[3]=float(data.range)
  #print("ir_4",str(data.range))

def callback_5(data):
  ir_array[4]=float(data.range)
  #print("ir_5",str(data.range))

def callback_6(data):
  ir_array[5]=float(data.range)
  #print("ir_6",str(data.range))

def callback_7(data):
  ir_array[6]=float(data.range)
  #print("ir_7",str(data.range))

def callback_8(data):
  ir_array[7]=float(data.range)
  #print("ir_8",str(data.range))
  #print(ir_array)

def listener():
  
  # In ROS, nodes are uniquely named. If two nodes with the same
  # name are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('listner', anonymous=True)

  rospy.Subscriber("sensor/ir_1", Range , callback_1)
  rospy.Subscriber("sensor/ir_2", Range , callback_2)
  rospy.Subscriber("sensor/ir_3", Range , callback_3)
  rospy.Subscriber("sensor/ir_4", Range , callback_4)
  rospy.Subscriber("sensor/ir_5", Range , callback_5)
  rospy.Subscriber("sensor/ir_6", Range , callback_6)
  rospy.Subscriber("sensor/ir_7", Range , callback_7)
  rospy.Subscriber("sensor/ir_8", Range , callback_8)
  # spin() simply keeps python from exiting until this node is stopped
  talker()
  rospy.spin()

def talker():
  pub = rospy.Publisher('ir_arr',Float32MultiArray, queue_size=10)
  #rospy.init_node('talker', anonymous=True)
  msg = Float32MultiArray()
  rate = rospy.Rate(10) # 10hz
  msg.data = ir_array
  while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
  listener()
  
  
  