#!/usr/bin/env python2
import smbus	
import time	
import rospy
import py_qmc5883l
from geometry_msgs.msg import Point

sensor = py_qmc5883l.QMC5883L()

def compass():
    pub = rospy.Publisher('magnet', Point, queue_size=10)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        # compass_str = "Compass deviation: %s" % sensor.get_magnet()
        d = sensor.get_data()
        magnetometer = Point(d[0],d[1],d[2])
        pub.publish(magnetometer)
        rate.sleep()

if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)
        compass()
    except rospy.ROSInterruptException:
        pass
