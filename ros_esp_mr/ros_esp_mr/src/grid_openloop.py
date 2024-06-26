#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_x_linear(velocity, duration):
    # Create a publisher for the Twist messages
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) 

    # Create a Twist message
    twist_msg = Twist()
    twist_msg.linear.x = velocity

    # Record the starting time
    start_time = time.time()

    # Publish the Twist message repeatedly to execute the linear movement for a certain duration
    while not rospy.is_shutdown() and time.time() - start_time < duration:
        pub.publish(twist_msg)
        rate.sleep()

    # Stop the robot 
    twist_msg.linear.x = 0.0
    pub.publish(twist_msg)

def move_z_angular(angular_velocity, duration):
    # Create a publisher for the Twist messages
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) 

    # Create a Twist message
    twist_msg = Twist()
    twist_msg.angular.z = angular_velocity

    # Record the starting time
    start_time = time.time()

    # Publish the Twist message repeatedly to execute the angular movement for a certain duration
    while not rospy.is_shutdown() and time.time() - start_time < duration:
        pub.publish(twist_msg)
        rate.sleep()

    # Stop the robot 
    twist_msg.angular.z = 0.0
    pub.publish(twist_msg)

if __name__ == '__main__':
    try:
         # Define the movement sequence
        move_x_linear(0.5, 2.5)
        rospy.sleep(1)
        move_z_angular(-0.5, 1.8)
        rospy.sleep(1) 
        move_x_linear(0.1, 1.5)
        rospy.sleep(1)
        move_z_angular(-0.5, 1.8)
        rospy.sleep(1)
        move_x_linear(0.5, 2.6)
        rospy.sleep(1)
        move_z_angular(0.5, 1.9)
        rospy.sleep(1)
        move_x_linear(0.1, 1.7)
        rospy.sleep(1)
        move_z_angular(0.5, 1.9)
        rospy.sleep(1)
        move_x_linear(0.5, 2.5)

        rospy.loginfo("Cleaning Task Completed.")

    except rospy.ROSInterruptException:
        pass
