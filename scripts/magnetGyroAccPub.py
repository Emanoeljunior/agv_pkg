#!/usr/bin/env python2

import smbus	
import rospy
from geometry_msgs.msg import Point
import py_mpu6050

def compass():
    pub = rospy.Publisher('magnet', Point, queue_size=10)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # compass_str = "Compass deviation: %s" % sensor.get_magnet()
        d = sensor.get_data()
        magnetometer = Point(d[0],d[1],d[2])
        pub.publish(magnetometer)
        rate.sleep()

def accelerometer_gyroscope(bus):
    
    acc, gyro = acc_gyro.get_data()
    Acc = Point(acc["x"], acc["y"], acc["z"])
    Gyro = Point(gyro["x"], gyro["y"], gyro["z"])
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    gyroPub.publish(Gyro)
    accPub.publish(Acc)

if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(bus.close)
    acc_gyro = py_mpu6050.MPU6050(bus)
    
    # ROS LOOP
    rospy.init_node("Sensors")
    rate = rospy.Rate(10)  # 60hz

    while not rospy.is_shutdown():
        accelerometer_gyroscope(acc_gyro)
        rate.sleep()