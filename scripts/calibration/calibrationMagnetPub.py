#!/usr/bin/env python2
import smbus	
import time	
import rospy
import py_qmc5883l
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import tf
import math

class Calibration():
    
    def __init__(self):
        max = 32767 # Max for signed 16 bits sensor
        min = - max # Min for signed 16 bits sensor
        self.max = Point(min,min,min) # Set max as min for start condition
        self.min = Point(max,max,max)  # Set min as max for start condition
        self.offset = Point(0,0,0)
        
        rospy.init_node('compass', anonymous=True)
        
        self.magnet_pub = rospy.Publisher('magnet', Point, queue_size=10)
        self.magnet_bearing_pub = rospy.Publisher('magnet_bearning', Float64, queue_size=10)
        
        self.magnet_max_pub = rospy.Publisher('magnet_max', Point, queue_size=10)
        self.magnet_min_pub = rospy.Publisher('magnet_min', Point, queue_size=10)
        self.magnet_offset_pub = rospy.Publisher('magnet_offset', Point, queue_size=10)
        
        self.ref_pub = rospy.Publisher('reference', geometry_msgs.msg.Point, queue_size=10)
        self.br = tf.TransformBroadcaster()


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
        print("Running..")
        while not rospy.is_shutdown():
            d = sensor.get_data()
            bearing = sensor.get_bearing_raw()
            magnetometer = Point(d[0],d[1],d[2])
            self.get_offset(magnetometer)
            self.magnet_pub.publish(magnetometer)
            self.magnet_bearing_pub.publish(bearing)
            rate.sleep()
    def end(self):
        print("Max ", self.max)
        print("Min ", self.min)
        print("Offset ", self.offset)
        print "Exited calibration magnet"
        bus.close()
        
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
        bus = smbus.SMBus(1)
        sensor = py_qmc5883l.QMC5883L(bus)
        calibration = Calibration()
        rospy.on_shutdown(calibration.end)
        calibration.compass(sensor)
    except rospy.ROSInterruptException:
        pass        
