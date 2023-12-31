#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt


class MovingAverage :
    
    def __init__ (self, sub, pub):
        self.points = []
        self.mean = Point(0,0,0)
        self.sub = rospy.Subscriber(sub, Point, self.moving_avarage, queue_size = 20)
        self.pub = rospy.Publisher(pub, Point, queue_size=20)

    def moving_avarage(self, data):
        self.points.append(data)
        if len(self.points)>20:
            old_point = self.points.pop(0)
            self.mean.x += (data.x - old_point.x)/20
            self.mean.y += (data.y - old_point.y)/20
            self.mean.z += (data.z - old_point.z)/20
            
        else:
            self.mean.x = sum(point.x for point in self.points)/len(self.points)
            self.mean.y = sum(point.y for point in self.points)/len(self.points)
            self.mean.z = sum(point.z for point in self.points)/len(self.points)

        self.pub.publish(self.mean)
    

if __name__ == '__main__':
    try:
        rospy.init_node("acc_filter")
        MovingAverage("/gyroscope_publisher","/gyroscope_mean_publisher")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         
