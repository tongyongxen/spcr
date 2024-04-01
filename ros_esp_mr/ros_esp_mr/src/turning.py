#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def turn(turning_duration):
    rospy.init_node('turn_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(20)
    twist_msg = Twist()
    twist_msg.angular.z = 0.5  
    start_time = time.time()

    while not rospy.is_shutdown() and time.time() - start_time < turning_duration:
        pub.publish(twist_msg)
        rate.sleep()

    twist_msg.angular.z = 0.0
    pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        turning_duration = 2.17
        turn(turning_duration)
    except rospy.ROSInterruptException:
        pass
