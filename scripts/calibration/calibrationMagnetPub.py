#!/usr/bin/env python2
import smbus	
import time	
import rospy
import py_qmc5883l
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class Calibration():
    
    def __init__(self):
        self.max = Point(0,0,0)
        self.min = Point(0,0,0)
        self.ofsset = Point(0,0,0)
        
        rospy.init_node('compass', anonymous=True)
        
        self.magnet_pub = rospy.Publisher('magnet', Point, queue_size=10)
        self.magnet_bearing_pub = rospy.Publisher('magnet_bearning', Float64, queue_size=10)
        
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
        while not rospy.is_shutdown():
            d = sensor.get_data()
            print(d)
            bearing = sensor.get_bearing_raw()
            magnetometer = Point(d[0],d[1],d[2])
            self.get_offset(magnetometer)
            self.magnet_pub.publish(magnetometer)
            self.magnet_bearing_pub.publish(bearing)
            rate.sleep()
    def end(self):
        print "Exited calibration magnet"
        bus.close()
        


if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)
        sensor = py_qmc5883l.QMC5883L(bus)
        calibration = Calibration()
        rospy.on_shutdown(calibration.end)
        calibration.compass(sensor)
    except rospy.ROSInterruptException:
        pass        
