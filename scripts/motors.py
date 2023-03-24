#!/usr/bin/python3 -tt
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

import time

class Motors():
    def __init__(self):


        rospy.init_node("motors")

        self.PWMA = 32
        self.AIN2 = 13
        self.AIN1 = 15
        self.BIN1 = 16
        self.BIN2 = 18
        self.PWMB = 33

        self.ENCA1 = 36
        self.ENCA2 = 38
        self.ENCB1 = 37
        self.ENCB2 = 40

        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.PWMB, GPIO.OUT)

        GPIO.setup(self.ENCA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        #GPIO.add_event_detect(self.ENCA2, GPIO.RISING, callback=self.encoderCountLeft)
        #GPIO.add_event_detect(self.ENCB2, GPIO.BOTH, callback=self.encoderCountRight)

        self.pwmA = GPIO.PWM(self.PWMA, 500)
        self.pwmB = GPIO.PWM(self.PWMB, 500)

        self.speedMotorLeft = 100
        self.sppedMotorRight = 100

        self.CPR = 48

        self.ticksCountLeft = 0
        self.ticksCountRight = 0
        self.lastEncA2 = 0
        self.lastEncB2 = 0

        self.encoderCountLeft = 0
        self.encoderCountRight = 0


        GPIO.output(self.AIN1, 1)
        GPIO.output(self.AIN2, 0)

        self.motorValues = [1, 0, 0, 1, 0, 0]   #AIN1, AIN2, PWMA, BIN1, BIN2, PWMB

        self.max_speed = 10
        self.min_speed = 0
        self.max_dutyCycle = 100
        self.min_dutyCycle = 0

        self.pwmA.start(0)
        self.pwmB.start(0)
        rospy.Subscriber("/cmd_vel", String, self.vel_callback)

    
    def vel_callback(self, msg):
        left_speed, right_speed = list(map(float,msg.data.split(',')))
        self.motorValues[2] = (left_speed - self.min_speed)/(self.max_speed - self.min_speed) * (self.max_dutyCycle - self.min_dutyCycle) + self.min_dutyCycle
        self.motorValues[5] = (right_speed - self.min_speed)/(self.max_speed - self.min_speed) * (self.max_dutyCycle - self.min_dutyCycle) + self.min_dutyCycle

        if left_speed > 0:
            self.motorValues[0] = 1
            self.motorValues[1] = 0
        else:
            self.motorValues[0] = 0
            self.motorValues[1] = 1
        
        if right_speed > 0:
            self.motorValues[3] = 1
            self.motorValues[4] = 0
        else:
            self.motorValues[3] = 0
            self.motorValues[4] = 1
        

        print("left speed:", left_speed, "   right_speed:", right_speed, "   left pwm:", self.motorValues[2], "   right pwm:", self.motorValues[5])



    def setMotorValues(self):
        GPIO.output(self.AIN1, self.motorValues[0])
        GPIO.output(self.AIN2, self.motorValues[1])
        self.pwmA.ChangeDutyCycle(abs(self.motorValues[2]))
        GPIO.output(self.BIN1, self.motorValues[3])
        GPIO.output(self.BIN2, self.motorValues[4])
        self.pwmB.ChangeDutyCycle(abs(self.motorValues[5]))

    def forward(self):
        self.motorValues = [1, 0, 70, 1, 0, 70]
        self.setMotorValues()
    
    def encoderCountLeft(self, channel): 
        enc_A1_state = GPIO.input(self.ENCA1)
        enc_A2_state = GPIO.input(self.ENCA2)
        print(enc_A2_state)
        # if enc_A2_state != self.lastEncA2:
        #     if enc_A1_state == self.lastEncA2:
        #         self.ticksCountLeft  += 1
        #     else:
        #         self.ticksCountLeft  -= 1
        #     self.lastEncA2 = enc_A2_state
        if enc_A1_state == 1:
            self.ticksCountLeft  += 1
        else:
            self.ticksCountLeft  -= 1
        print("left encoder: ", self.ticksCountLeft)  
        # time.sleep(0.2) 

    def encoderCountRight(self, channel):
        enc_B1_state = GPIO.input(self.ENCB1)
        enc_B2_state = GPIO.input(self.ENCB2)
        if enc_B2_state != self.lastEncB2:
            if enc_B1_state == self.lastEncB2:
                self.ticksCountRight  += 1
            else:
                self.ticksCountRight  -= 1
            self.lastEncB2 = enc_B2_state
        print("right encoder: ", self.ticksCountRight) 



if __name__ == "__main__":
  motors = Motors()
  try:
    while not rospy.is_shutdown():
        motors.setMotorValues()
  except rospy.ROSInterruptException:
    pass
        # motors.setMotorValues()
#        rospy.spin()


        # enc_A1_state = GPIO.input(motors.ENCA1)
        # enc_A2_state = GPIO.input(motors.ENCA2)
        # print(enc_A1_state, enc_A2_state)
        # if enc_A2_state != motors.lastEncA2:
        #     if enc_A2_state == 1:
        #         if enc_A1_state == 0:
        #             motors.encoderCountLeft -= 1
        #         else:
        #             motors.encoderCountLeft += 1
        #     motors.lastEncA2 = enc_A2_state
        # print("left motor encoder count:", motors.encoderCountLeft)
        # time.sleep(0.08)