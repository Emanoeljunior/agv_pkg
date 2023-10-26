#!/usr/bin/env python2

import smbus	
import rospy
from geometry_msgs.msg import Point

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(bus.close)



    # ROS LOOP
    rospy.init_node("MPU6050")
    rate = rospy.Rate(30)  # 60hz
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    while not rospy.is_shutdown():
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)	
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)	
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