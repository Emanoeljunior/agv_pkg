#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
import smbus	
import rospy
from geometry_msgs.msg import Point
import py_mpu6050

def end():
    print "Exited calibration Accelerometer and gyroscope"
    bus.close()

def acc_gyro_read_pub(acc_gyro):
    
    acc, gyro = acc_gyro.get_data()
    Acc = Point(acc["x"], acc["y"], acc["z"])
    Gyro = Point(gyro["x"], gyro["y"], gyro["z"])
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    gyroPub.publish(Gyro)
    accPub.publish(Acc)


if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(end)
    acc_gyro = py_mpu6050.MPU6050(bus)
    
    rospy.init_node("MPU6050_CALIBRATION")
    rate = rospy.Rate(2) #Put 60 as default
    # ROS LOOP
    while not rospy.is_shutdown(): 
        acc_gyro_read_pub(acc_gyro)
        rate.sleep()
