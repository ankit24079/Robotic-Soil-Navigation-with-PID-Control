#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node("obstacle_pub_node")
        rospy.logwarn("start running obstacle_pub_node")
        rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.Subscriber("/pid_cmd_vel", Twist, self.pid_callback)

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.latest_pid_cmd = Twist()
        self.obstacle = False


    def callback(self, msg):
        if (msg.ranges[0] > 0.5 and msg.ranges[25] > 0.5 and msg.ranges[335] > 0.5):
            self.obstacle = False
        else:
            self.obstacle = True
        
        self.publish_velocity()


    def pid_callback(self, msg):
        self.latest_pid_cmd = msg
        self.publish_velocity()


    def publish_velocity(self):
        cmd = Twist()
        if self.obstacle:
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
        else:
            cmd = self.latest_pid_cmd
        
        self.pub.publish(cmd)
    

    def run(self):
        rospy.spin()
        

if __name__=="__main__":
    obs = ObstacleAvoidance()
    obs.run()
