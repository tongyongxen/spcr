#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

def control_brush_motor():
    rospy.init_node('brush_control_node', anonymous=True)
    pub = rospy.Publisher('brush_cmd', Empty, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Toggle the brush motor state
        pub.publish(Empty())
        rospy.loginfo("Toggle brush motor state")
        rate.sleep()

if __name__ == '__main__':
    try:
        control_brush_motor()
    except rospy.ROSInterruptException:
        pass
