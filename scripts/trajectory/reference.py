#!/usr/bin/env python3
import time	
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import tf
import math

class Trajectory():
    
    def __init__(self):
        rospy.init_node('trajectory', anonymous=True)
        
        self.ref_pub = rospy.Publisher('reference', Point, queue_size=10)
        self.br = tf.TransformBroadcaster()

    def reference(self):
        # Doing the circle reference         
        t = rospy.Time.now().to_sec() * math.pi            
        x = 2.0 * math.cos(t/70)            
        y = 2.0 * math.sin(t/70)      # Create a child frame of odom for see the reference in RVIZ           
        self.br.sendTransform(
        [ x, y, 0.0],                            
        [0.0, 0.0, 0.0, 1.0],
        rospy.Time.now(),
        "reference",
        "odom")      # Send the reference into topic                  
        reference = Point()            
        reference.x = x            
        reference.y = y            
        self.ref_pub.publish(reference) 
        


if __name__ == '__main__':
    try:
        trajectory = Trajectory()
        while not rospy.is_shutdown():
            self.reference()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass        
