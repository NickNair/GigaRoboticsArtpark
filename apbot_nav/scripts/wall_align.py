#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf import transformations
from nav_msgs.msg import Odometry

isAligned = False
isDistfixed = False

class WallAlign():
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.get_laser_data, queue_size=100, tcp_nodelay=True)  
        self.data = [] 
        rospy.sleep(2)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.t0 = 0
        self.tick = 0

    def align(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        angular_z = 0

        min_dist = min(self.data.ranges[170:189])
        min_index = self.data.ranges.index(min(self.data.ranges[170:189]))

        t1 = rospy.get_time()
        if t1 - self.t0 > 10:
            print("done by time")
            vel.angular.z = 0
            self.pub.publish(vel)
            return True

        # if min_dist == self.data.ranges[179]:
        #     if self.tick >= 20:
        #         vel.angular.z = 0
        #         self.pub.publish(vel)
        #         return True
        #     else:
        #         self.tick += 1

        else:
            vel.angular.z = .01*(min_index-179)
            self.pub.publish(vel)
            return False

    def dist_fix(self):
        vel = Twist()
        if abs(self.data.ranges[180] - .7) > .05:
            vel.linear.x = .5*(self.data.ranges[180] - .75)   
            self.pub.publish(vel)
            return False     
        else:
            vel.linear.x = 0
            self.pub.publish(vel)
            return True

    def get_laser_data(self, data):
        self.data = data

if __name__=="__main__":
    rospy.init_node('find_door')
    wa = WallAlign()
    t = False
    wa.t0 = rospy.get_time()
    while not t:
        t = wa.align()
    print("done")
    status_pub = rospy.Publisher('/align_status', String, queue_size=10)
    status_pub.publish("Not Aligned")
    # Velocity publisher
    rospy.spin()