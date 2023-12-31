#!/usr/bin/env python2
# import rosbag
import smbus	
import time	
import rospy
import py_qmc5883l
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class Calibration():
    
    def __init__(self):
        max = 32767 # Max for signed 16 bits sensor
        min = - max # Min for signed 16 bits sensor
        self.max = Point(min,min,min) # Set max as min for start condition
        self.min = Point(max,max,max)  # Set min as max for start condition
        self.offset = Point(0,0,0)
        # self.bag = rosbag.Bag('magnetometer_calibration.bag', 'w')
        
        rospy.init_node('compass', anonymous=True)
        
        self.magnet_pub = rospy.Publisher('magnet', Point, queue_size=10)
        self.magnet_bearing_pub = rospy.Publisher('magnet_bearning', Float64, queue_size=10)
        self.magnet_calibrated_pub = rospy.Publisher('magnet_calibrated', Point, queue_size=10)
        
        self.magnet_max_pub = rospy.Publisher('magnet_max', Point, queue_size=10)
        self.magnet_min_pub = rospy.Publisher('magnet_min', Point, queue_size=10)
        self.magnet_offset_pub = rospy.Publisher('magnet_offset', Point, queue_size=10)
        

    def get_offset(self, data):
        if data.x > self.max.x:
            self.max.x = data.x
        if data.y > self.max.y:
            self.max.y = data.y
        if data.z > self.max.z:
            self.max.z = data.z
            
        if data.x < self.min.x:
            self.min.x = data.x
        if data.y < self.min.y:
            self.min.y = data.y
        if data.z < self.min.z:
            self.min.z = data.z
            
        self.offset.x = (self.max.x + self.min.x)/2
        self.offset.y = (self.max.y + self.min.y)/2
        self.offset.z = (self.max.z + self.min.z)/2
        
        self.magnet_max_pub.publish(self.max)
        self.magnet_min_pub.publish(self.min)
        self.magnet_offset_pub.publish(self.offset)
        

    def compass(self, sensor):
       
        rate = rospy.Rate(10) # 10hz
        print("Running..")
        

        while not rospy.is_shutdown():
            d = sensor.get_data()
            magnetometer = Point(d[0],d[1],d[2])
            d_c = self.calibrated(d[:3])
            calibrated = Point(d_c[0],d_c[1],d_c[2])
            bearing = sensor.get_bearing(d_c[0],d_c[1])
            self.get_offset(magnetometer)
            self.magnet_pub.publish(magnetometer)
            self.magnet_bearing_pub.publish(bearing)
            self.magnet_calibrated_pub.publish(calibrated)
            # self.bag.write('magnet_pub_bag', magnetometer)
            rate.sleep()
    def end(self):
        print("Max ", self.max)
        print("Min ", self.min)
        print("Offset ", self.offset)
        print "Exited calibration magnet"
        # self.bag.close()
        
        bus.close()
        
    def calibrated(self, data):
        
        b = np.array([[5544.88247866],
                      [131.46512601],
                      [9.97424514]])

        A_1 = np.array([[3.60496438e-04, -1.43519561e-05, 1.89520948e-06],
                        [-1.43519561e-05, 3.15399830e-04, 3.06530881e-06],
                        [1.89520948e-06, 3.06530881e-06, 3.99551985e-04]])
        
        # A_1 = np.array(
        #     [[ 3.47544601e-04, -9.31171840e-07, -8.71559471e-06],
        #      [-9.31171840e-07,  3.26957519e-04,  1.87713564e-05],
        #      [-8.71559471e-06,  1.87713564e-05,  3.95850480e-04]])
        # b = np.array(
        #     [[5547.74334509],
        #      [ 0],
        #      [ 110.55955971]]
        # )
            # b
            # [[5547.74334509]
            #  [ 828.87628051]
            #  [ 110.55955971]]
            # A_1
            # [[ 3.47544601e-04 -9.31171840e-06 -8.71559471e-07]
            #  [-9.31171840e-06  3.26957519e-04  1.87713564e-05]
            #  [-8.71559471e-07  1.87713564e-05  3.95850480e-04]]
            


        s = np.array(data).reshape(3, 1)
        s = np.dot(A_1, s - b)
        return [s[0,0], s[1,0], s[2,0]]


if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)
        sensor = py_qmc5883l.QMC5883L(bus)
        calibration = Calibration()
        rospy.on_shutdown(calibration.end)
        calibration.compass(sensor)
    except rospy.ROSInterruptException:
        pass        
