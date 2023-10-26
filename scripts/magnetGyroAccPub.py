#!/usr/bin/env python2

import smbus	
import rospy
from geometry_msgs.msg import Point
import py_mpu6050


if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(bus.close)
    sensor = py_mpu6050.MPU6050()

    # ROS LOOP
    rospy.init_node("MPU6050")
    rate = rospy.Rate(30)  # 60hz
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    while not rospy.is_shutdown():
        acc_x = sensor.read_raw_data(ACCEL_XOUT_H)
        acc_y = sensor.read_raw_data(ACCEL_YOUT_H)
        acc_z = sensor.read_raw_data(ACCEL_ZOUT_H)	
        gyro_x = sensor.read_raw_data(GYRO_XOUT_H)
        gyro_y = sensor.read_raw_data(GYRO_YOUT_H)
        gyro_z = sensor.read_raw_data(GYRO_ZOUT_H)	
        Ax = (acc_x/16384.0-0.101778)*9.81 #16384 -> 1g 
        Ay = (acc_y/16384.0+0.007739)*9.81
        Az = (acc_z/16384.0-0.987720)*9.81	
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        Acc = Point(Ax, Ay, Az)
        Gyro = Point(Gx, Gy, Gz)
        gyroPub.publish(Gyro)
        accPub.publish(Acc)
        rate.sleep()