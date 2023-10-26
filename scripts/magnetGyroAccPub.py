#!/usr/bin/env python2

import smbus	
import rospy
from geometry_msgs.msg import Point
import py_mpu6050


if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(bus.close)
    sensor = py_mpu6050.MPU6050(bus)

    # ROS LOOP
    rospy.init_node("MPU6050")
    rate = rospy.Rate(30)  # 60hz
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    while not rospy.is_shutdown():
	
        acc, gyro = sensor.get_data()
        Acc = Point(acc["x"], acc["y"], acc["z"])
        Gyro = Point(gyro["x"], gyro["y"], gyro["z"])
        gyroPub.publish(Gyro)
        accPub.publish(Acc)
        rate.sleep()