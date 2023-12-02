#!/usr/bin/env python2
import time
import smbus	
import rospy
from geometry_msgs.msg import Point
import py_mpu6050
import py_qmc5883l
import kalman

class SensorsRead:
    
    def __init__(self, current_time):
        self.current_time = current_time
        self.compass_data = None
        self.acc_data = None
        self.gyro_data = None
        

    def compass_read_pub(self, compass):
        pub = rospy.Publisher('magnet', Point, queue_size=10)
        self.compass_data = compass.get_calibrated_magnet_data()
        magnetometer = Point( self.compass_data[0], self.compass_data[1], self.compass_data[2])
        pub.publish(magnetometer)

    def acc_gyro_read_pub(self, acc_gyro):
        
        self.acc_data, self.gyro_data = acc_gyro.get_data()
        Acc = Point(self.acc_data["x"], self.acc_data["y"], self.acc_data["z"])
        Gyro = Point(self.gyro_data["x"], self.gyro_data["y"], self.gyro_data["z"])
        gyroPub = rospy.Publisher('/gyroscope_publisher', Point, queue_size=60)
        accPub = rospy.Publisher('/accelerometer_publisher', Point, queue_size=120)
        gyroPub.publish(Gyro)
        accPub.publish(Acc)
    
    def apply_kalman_filter(self, sensorfusion):
        new_time = time.time()
        dt = new_time - self.current_time
        self.current_time = new_time
        sensorfusion.computeAndUpdateRollPitchYaw(self.acc_data["x"],self.acc_data["y"],self.acc_data["z"],self.gyro_data["x"],self.gyro_data["y"], self.gyro_data["z"],self.compass_data[0], self.compass_data[1], self.compass_data[2], dt)
        kalmanPub = rospy.Publisher('/kalman_publisher', Point, queue_size=120)
        kalmanData = Point(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw)
        kalmanPub.publish(kalmanData)
        
        
    
def end():
    print "end"
    bus.close()

if __name__ == '__main__':
    bus = smbus.SMBus(1)
    rospy.on_shutdown(end)
    acc_gyro = py_mpu6050.MPU6050(bus)
    compass = py_qmc5883l.QMC5883L(bus)
    current_time = time.time()
    sensors = SensorsRead(current_time)
    sensorfusion = kalman.Kalman()
    
    # ROS LOOP
    rospy.init_node("Sensors")
    rate = rospy.Rate(10)  # 60hz
    while not rospy.is_shutdown():
        sensors.acc_gyro_read_pub(acc_gyro)
        sensors.compass_read_pub(compass)
        sensors.apply_kalman_filter(sensorfusion)
        rate.sleep()
