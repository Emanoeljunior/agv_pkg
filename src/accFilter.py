#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt


class GyroAccc :
    
    def __init__ (self, sub, pub):
        self.points = []
        # self.counter = 0
        self.mean = Point(0,0,0)
        self.gyro_acc_sub = rospy.Subscriber(sub, Point, self.moving_avarage, queue_size = 20)
        self.gyro_mean_pub = rospy.Publisher(pub, Point, queue_size=20)

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
        # print("Report")
        # print("Length:", len(self.points))
        # print("Mean", self.mean)
        self.gyro_mean_pub.publish(self.mean)
        self.plot_x(self.mean)
    
    # def plot_x(self,msg):
    #     if self.counter % 2 == 0:
    #         plt.plot(msg.y, msg.x, '*')
    #         plt.axis("equal")
    #         plt.draw()
    #         plt.pause(0.00000000001)

    #     self.counter += 1
        

if __name__ == '__main__':
    try:
        rospy.init_node("acc_filter")
        GyroAccc("/gyroscope_publisher","/gyroscope_mean_publisher")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         
