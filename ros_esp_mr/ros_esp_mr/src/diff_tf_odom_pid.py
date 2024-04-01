#!/usr/bin/env python

import rospy
import roslib
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class DiffTf:
    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        # Parameters
        self.rate = rospy.get_param('~rate', 10.0)
        self.ticks_meter = float(rospy.get_param('ticks_meter', 12105))
        self.base_width = float(rospy.get_param('~base_width', 0.245))
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # Internal data
        self.enc_left = None
        self.enc_right = None
        self.left = 0
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()
        
        # Subscriptions
        rospy.Subscriber("enc_l_data", Int16, self.lwheelCallback)
        rospy.Subscriber("enc_r_data", Int16, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        # PID controllers for linear and angular velocities
        self.linear_pid = PID(1.0, 0.1, 0.05)  # Example PID parameters, tune as needed
        self.angular_pid = PID(1.0, 0.1, 0.05)  # Example PID parameters, tune as needed
    
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # Calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            d = (d_left + d_right) / 2
            th = (d_right - d_left) / self.base_width
            
            self.dx = d / elapsed
            self.dr = th / elapsed
           
            if (d != 0):
                x = cos(th) * d
                y = -sin(th) * d
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if(th != 0):
                self.th = self.th + th
                
            # Update PID controllers
            target_linear_velocity = 0.5  # Adjust as needed
            target_angular_velocity = 0.2  # Adjust as needed
            linear_error = target_linear_velocity - self.dx
            angular_error = target_angular_velocity - self.dr
            linear_cmd = self.linear_pid.update(linear_error, elapsed)
            angular_cmd = self.angular_pid.update(angular_error, elapsed)
            
            # Update velocities using PID outputs
            self.dx += linear_cmd
            self.dr += angular_cmd
            
            # Publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
    def lwheelCallback(self, msg):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    def rwheelCallback(self, msg):
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

if __name__ == '__main__':
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass

