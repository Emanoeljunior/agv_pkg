#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
import RPi.GPIO as GPIO

"""
A1 = 16
A2 = 20
AEn = 13

B1 = 26
B2 = 21
BEn = 19
#"""
#GPIO.setwarnings(False)
class Motor :
    def __init__ (self, pin1, pin2, pinEn, sub):
        self.pin1 = pin1
	self.pin2 = pin2
	self.pinEn = pinEn
	self.motor_sub = rospy.Subscriber(sub, Float64, self.updateMotorCallback, queue_size = 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
	GPIO.setup(self.pinEn, GPIO.OUT)
        self.Motor_pwm = GPIO.PWM(self.pinEn, 120)
        self.Motor_pwm.start(0)
    def updateMotorCallback(self, msg):
        self.power = msg.data
        self.duty = abs(self.power)*100
        if self.power>0:
            GPIO.output(self.pin1, True)
            GPIO.output(self.pin2, False)
        else :
            GPIO.output(self.pin1, False)
            GPIO.output(self.pin2, True)
        self.Motor_pwm.ChangeDutyCycle(self.duty)

if __name__ == '__main__':
    try:
        rospy.init_node("motor_node")
        #left motor
        A1 = 16
        A2 = 20
        AEn = 13
        #right motor
        B1 = 26
        B2 = 21
        BEn = 19
        Motor(A1, A2, AEn, "/left_motor")
        Motor(B1, B2, BEn, "/right_motor")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         
