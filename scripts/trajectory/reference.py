#!/usr/bin/env python3
import time	
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math

class Trajectory():
    
    def __init__(self):
        rospy.init_node('trajectory', anonymous=True)
        
        self.ref_pub = rospy.Publisher('reference', Point, queue_size=10)

    def reference(self):
        # Doing the circle reference         
        t = rospy.Time.now().to_sec() * math.pi*10           
        x = 1.0 * math.cos(t/70)            
        y = 1.0 * math.sin(t/70)               
        reference = Point()            
        reference.x = x            
        reference.y = y            
        self.ref_pub.publish(reference) 
        


if __name__ == '__main__':
    try:
        trajectory = Trajectory()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            trajectory.reference()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass        
