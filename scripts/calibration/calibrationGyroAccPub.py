#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
import smbus	
import rospy
from geometry_msgs.msg import Point

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit -> [0-65536]
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
    Device_Address = 0x68

    # Register addresses 
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    # MPU6050 configuration
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 0)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 0)

    # ROS LOOP
    rospy.init_node("MPU6050")
    rate = rospy.Rate(60)
    gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
    accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
    while not rospy.is_shutdown():
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)	
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)	
        Ax = (acc_x/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Ax[m/s²]
        Ay = (acc_y/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Ay[m/s²]    
        Az = (acc_z/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Az[m/s²]	
        Gx = gyro_x/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gx[degres/s]
        Gy = gyro_y/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gy[degres/s]
        Gz = gyro_z/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gz[degres/s]
        Acc = Point(Ax, Ay, Az)
        Gyro = Point(Gx, Gy, Gz)
        gyroPub.publish(Gyro)
        accPub.publish(Acc)
        rate.sleep()