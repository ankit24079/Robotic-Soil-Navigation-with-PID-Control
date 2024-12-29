#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import math
import time
import csv


class PIDControl:
    def __init__(self):
        rospy.init_node("pid_pub_node")
        rospy.logwarn("start running pid_pub_node")

        self.p_linear = 0.10
        self.i_linear = 0.01
        self.d_linear = 0.0

        self.p_angular = 1.0
        self.i_angular = 0.01
        self.d_angular = 0.1

        self.x_end = 0.0
        self.y_end = -7.0

        self.previous_err_dist = 0.0
        self.previous_err_angle = 0.0
        self.integral_dist = 0.0
        self.integral_angle = 0.0


        self.start_time = time.time()
        rospy.logwarn("Opening file")
        self.file = open("../csci5551_project/pid_data/pid_control_data11.csv", "w")
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(["Time", "X Position", "Y Position", "Error", "Linear Velocity", "Angular Velocity"])

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.pub = rospy.Publisher("/pid_cmd_vel", Twist, queue_size = 1)
        self.rate = rospy.Rate(10)


    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
        x_error = self.x_end - x
        y_error = self.y_end - y

        dist_err = math.sqrt(x_error**2 + y_error**2)
        angle_to_goal = math.atan2(y_error, x_error)
        angle_error = angle_to_goal - theta

        curr_time = time.time()
        if self.start_time:
            dt = curr_time - self.start_time
        else:
            dt = 1.0
        
        # self.start_time = curr_time

        self.integral_dist = self.integral_dist + dist_err * dt
        self.integral_angle = self.integral_angle + angle_error * dt

        derivative_dist = (dist_err - self.previous_err_dist) / dt
        derivative_angle = (angle_error - self.previous_err_angle) / dt

        self.previous_err_dist = dist_err
        self.previous_err_angle = angle_error

        cmd = Twist()
        cmd.linear.x = ((self.p_linear * dist_err) + (self.i_linear * self.integral_dist) + (self.d_linear * derivative_dist))
        cmd.angular.z = ((self.p_angular * angle_error) + (self.i_angular * self.integral_angle) + (self.d_angular * derivative_angle))

        self.pub.publish(cmd)
        self.csv_writer.writerow([dt, x, y, dist_err, cmd.linear.x, cmd.angular.z])


    def run(self):
        rospy.spin()


if __name__=="__main__":
    c = PIDControl()
    c.run()
    self.file.close()


