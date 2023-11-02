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

    def get_offset(self, data):
        if data.x > self.max.x:
            self.max.x = data.x
        print(self.max.x)

    def compass(self, sensor):
        magnet_pub = rospy.Publisher('magnet', Point, queue_size=10)
        magnet_bearing_pub = rospy.Publisher('magnet_bearning', Float64, queue_size=10)
        rospy.init_node('compass', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            d = sensor.get_data()
            print(d)
            self.get_offset(d)
            bearing = sensor.get_bearing_raw()
            magnetometer = Point(d[0],d[1],d[2])
            magnet_pub.publish(magnetometer)
            magnet_bearing_pub.publish(bearing)
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
