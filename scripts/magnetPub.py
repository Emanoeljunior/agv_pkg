#!/usr/bin/env python2
import smbus	
import time	
import rospy
import py_qmc5883l
from std_msgs.msg import String

sensor = py_qmc5883l.QMC5883L()

def compass():
    pub = rospy.Publisher('magnet', String, queue_size=10)
    rospy.init_node('compass', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        compass_str = "Compass deviation: %s" % sensor.get_magnet()
        d = sensor.get_data()
        compass_str = "Compass Data: [x: %s, y: %s, z: %s, t: %s]" % (d[0], d[1], d[2], d[3])
        #rospy.loginfo(type(sensor.get_magnet()))
        print compass_str
        pub.publish(compass_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        bus = smbus.SMBus(1)
        compass()
    except rospy.ROSInterruptException:
        pass
