#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ir_sensor_sub = rospy.Subscriber('/ir_sensor_status', Bool, self.ir_sensor_callback)
        self.obstacle_detected = False

    def ir_sensor_callback(self, data):
        self.obstacle_detected = data.data

    def stop_robot(self):
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)  

    def move_backward(self):
        backward_twist = Twist()
        backward_twist.linear.x = -0.2  
        self.cmd_vel_pub.publish(backward_twist)

    def run(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.move_backward() 
                while self.obstacle_detected: 
                    rate.sleep()
            else:
                self.stop_robot()
            rate.sleep()

if __name__ == '__main__':
    controller = RobotController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
