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
of the Accelerometer(acc) and Gyroscope(gyro) sensor on axis X, Y and Z, e.g. acc = [-0.90, 1.20, 9.81].
"""
    
import numpy as np

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
STANDBY      = 0x6C

class MPU6050(object):
    
    def __init__(self, bus):
        self.bus = bus
      
        # MPU6050 configuration
        self.bus.write_byte_data(Device_Address, SMPLRT_DIV, 0)
        self.bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        self.bus.write_byte_data(Device_Address, CONFIG, 0)
        self.bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        self.bus.write_byte_data(Device_Address, INT_ENABLE, 0)
    
    def stand_by(self):
        self.bus.write_byte_data(Device_Address, STANDBY, 63) #0b0011_1111
        
    def read_raw_data(self,addr):
	      #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(Device_Address, addr)
        low = self.bus.read_byte_data(Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
    def get_data(self):
        """
		Read measurements from accelerometer and gyroscope
  
        Configuration
        -------
        Accelerometer with 2g (2 gravity) of range
        and 16 bits.
        Gyroscope with 250°/s (250 degres per second) of range
        and 16 bits.
		Returns
		-------
		acc: dict["x": float,"y": float,"z": float]
				    Acceleration from each axis in m/s²
		gyro: dict["x": float,"y": float,"z": float]
					Angular velocity from each axis in degress/s
		"""
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)	
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)
        
        Ax = (acc_x/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Ax[m/s²]
        Ay = (acc_y/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Ay[m/s²]    
        Az = (acc_z/16384.0)*9.8  # 9.8 = 1g -> Scale is set in 2g = 32768 and -2g = -32768 -> Az[m/s²]	
        Gx = gyro_x/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gx[degres/s]
        Gy = gyro_y/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gy[degres/s]
        Gz = gyro_z/131.0 # -> Scale is set in 250°/s = 32768 and -250°/s = -32768 -> Gz[degres/s]
        
        return {"x":Ax,"y":Ay, "z":Az}, {"x":Gx, "y":Gy, "z":Gz}
    
    def get_roll_pitch(self, acc):
        """
		Computes measured roll and pitch from accelerometer
        source: https://medium.com/@niru5/intro-to-inertial-measurement-unit-imu-part-1-47f19fc7d68d
  

		Parameters
		----------
		ax: float 
			acceleration in x axis
		ay: float 
			acceleration in y axis
		az: float 
			acceleration in z axis

		Returns
		-------
		roll: float
					It is estimated roll from sensor values
		pitch: float
					It is estimated pitch from sensor values

		"""
        roll = np.degrees(np.arctan2(acc["y"], acc["z"]))
        pitch = np.degrees(np.arctan2(-1*acc["x"], np.sqrt(np.square(acc["y"]) + np.square(acc["z"]) )))
        return roll, pitch
    
    def __del__(self):
        print "MPU6050 deleted"
                
