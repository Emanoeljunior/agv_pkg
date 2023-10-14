#!/usr/bin/env python3

# Based on turblebot3_teleop

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, os
import tty, termios

MAX_LIN_VEL = 0.9

VEL_STEP_SIZE = 0.05

msg = """
Control Foxbot!
---------------------------
Moving around:
   w          i
   s   f  j   k
   

w/s : increase/decrease left motor velocity (Foxbot : ~ 0.9)
i/k : increase/decrease right motor velocity (Foxbot : ~ 0.9)

f   : stop left motor velocity
j   : stop right motor velocity

space key: force stop all

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_left_vel, target_right_vel):
    return "currently:\tleft vel %s\t right vel %s " % (target_left_vel,target_right_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return round(output, 3)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return round(input,3)

def checkLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
    return vel


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop')
    left_motor = rospy.Publisher('/left_motor', Float64, queue_size=10)
    right_motor = rospy.Publisher('/right_motor', Float64, queue_size=10)

    status = 0
    target_left_vel   = 0.0
    target_right_vel  = 0.0
    control_left_vel  = 0.0
    control_right_vel = 0.0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' :
                target_left_vel = checkLimitVelocity(target_left_vel + VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_left_vel,target_right_vel))
            elif key == 's' :
                target_left_vel = checkLimitVelocity(target_left_vel - VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_left_vel,target_right_vel))
            elif key == 'i' :
                target_right_vel = checkLimitVelocity(target_right_vel + VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_left_vel,target_right_vel))
            elif key == 'k' :
                target_right_vel = checkLimitVelocity(target_right_vel - VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_left_vel,target_right_vel))
            elif key == 'f':
                target_left_vel   = 0.0
                control_left_vel  = 0.0
                print(vels(target_left_vel, target_right_vel))
            elif key == 'j':
                target_right_vel  = 0.0
                control_right_vel = 0.0
                print(vels(target_left_vel, target_right_vel))
            elif key == ' ':
                target_left_vel   = 0.0
                control_left_vel  = 0.0
                target_right_vel  = 0.0
                control_right_vel = 0.0
                print(vels(target_left_vel, target_right_vel))
                

            if (key == '\x03'):
                break

            if status == 20 :
                print(msg)
                status = 0


            control_left_vel = makeSimpleProfile(control_left_vel, target_left_vel, (VEL_STEP_SIZE/2.0))

            control_right_vel = makeSimpleProfile(control_right_vel, target_right_vel, (VEL_STEP_SIZE/2.0))

            left_motor.publish(control_left_vel)
            right_motor.publish(control_right_vel)

    except:
        print(e)

    finally:
        left_motor.publish(0.0)
        right_motor.publish(0.0)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
