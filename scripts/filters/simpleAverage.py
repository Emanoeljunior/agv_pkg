#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt


class SimpleAverage :
    
    def __init__ (self, sub, pub):
        self.k = 0
        self.mean = Point(0,0,0)
        self.sub = rospy.Subscriber(sub, Point, self.simple_avarage, queue_size = 20)
        self.pub = rospy.Publisher(pub, Point, queue_size=20)

    def simple_avarage(self, data):
        self.k += 1

        self.mean.x += ((k-1)/k)*mean.x + (1/k)*data.x
        self.mean.y += ((k-1)/k)*mean.y + (1/k)*data.y
        self.mean.z += ((k-1)/k)*mean.z + (1/k)*data.z

        self.pub.publish(self.mean)
    

if __name__ == '__main__':
    try:
        rospy.init_node("acc_filter")
        SimpleAverage("/accelerometer_publisher","/acc_mean_pub")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         
