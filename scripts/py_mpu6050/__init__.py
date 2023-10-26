# -*- coding: utf-8 -*-
"""
Python driver for the MPU6050 3-Axis Accelerometer and Gyroscope Sensor.

Usage example:

  import py_mpu6050
  sensor = py_mpu6050.MPU6050()
  acc = sensor.get_acc()
  gyro = sensor.get_gyro()
  print(acc, gyro)

you will get three 16 bit signed integers, representing the values
of the Accelerometer(acc) and Gyroscope(gyro) sensor on axis X, Y and Z, e.g. acc = [-1257, 940, -4970].
"""
    

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

class MPU6050(object):
    
    def __init__(self, bus):
        self.bus = bus
      
        # MPU6050 configuration
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 0)
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        bus.write_byte_data(Device_Address, CONFIG, 0)
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        bus.write_byte_data(Device_Address, INT_ENABLE, 0)
        
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
