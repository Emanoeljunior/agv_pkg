#!/usr/bin/env python2
import smbus	
import time	
import rospy
import ..py_qmc5883l
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

sensor = py_qmc5883l.QMC5883L()

def compass():
    magnet_pub = rospy.Publisher('magnet', Point, queue_size=10)
    magnet_bearing_pub = rospy.Publisher('magnet_bearning', PoinFloat64t, queue_size=10)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        d = sensor.get_data()
        bearing = sensor.get_bearing_raw()
        magnetometer = Point(d[0],d[1],d[2])
        magnet_pub.publish(magnetometer)
        magnet_bearing_pub.publish(bearing)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        compass()
    except rospy.ROSInterruptException:
        pass        
