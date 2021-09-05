#!/usr/bin/env python3

import rospy
import math
import time
import py_trees
import py_trees.console as console

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from sensor_msgs.msg import LaserScan

# Global pose variables
global curr_x, curr_y, curr_z

# Initialize flag variables
getClose = True
isAligned = False
nearDoor = False
notInFront = True

# Laser callback
def get_regions(data):

    # print(data.ranges.index(min(min(data.ranges[90:300]), min(data.ranges[360:450]), min(data.ranges[510:570]), min(data.ranges[630:635]))))

    global getClose, isAligned

    # If both isn't close to the bathroom yet, get close
    if getClose:
        get_close(data)

    # If bot isn't aligned with the wall, align
    elif not isAligned:
        align_wall(data)

    # If bot is at the edge of the door
    elif not nearDoor:
        follow_wall(data)

    elif notInFront:
        get_in_front(data)

# Pose callback
def get_pose(data):

    global curr_x, curr_y, curr_z

    curr_x = data.pose.pose.position.x
    curr_y = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    curr_z = euler[2]

# Get distance between 2 points
def get_dist(x1,y1,x2,y2):

    return(math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2))

def get_in_front(laser_data):
    
    global notInFront

    # Velocity publisher
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    # Enter status publisher
    finish_pub = rospy.Publisher('/enter_done', String, queue_size=10)

    vel = Twist()

    # Turn to align
    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = -.1
    pub.publish(vel)
    rospy.sleep(1)
    vel.angular.z = 0
    pub.publish(vel)
    
    # Move right
    vel.linear.x = 0
    vel.linear.y = -0.2
    vel.angular.z = 0

    pub.publish(vel)

    rospy.sleep(1.75)

    # Move front
    vel.linear.x = .2
    vel.linear.y = 0
    vel.angular.z = 0

    pub.publish(vel)

    rospy.sleep(2.2)

    # Stop (Bathroom Entered)
    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = 0
    
    pub.publish(vel)

    finish_pub.publish("Done")
    
    notInFront = False

def follow_wall(laser_data):

    global nearDoor

    # Velocity publisher
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    vel = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    # Check which side the bot is facing relative to the wall
    if laser_data.ranges[180] > 2.5 and laser_data.ranges[180] < 5 and laser_data.ranges[90] < 3:
        nearDoor = True
        linear_x = 0
        linear_y = 0

    elif laser_data.ranges[180] < 10:
        min_laser_dist = min(laser_data.ranges[170:270])
        min_index = laser_data.ranges.index(min_laser_dist)
        rot_angle = (min_index - 180)/2
        angular_z = rot_angle*.01
        linear_y = -.2
        linear_x = (laser_data.ranges[180] - .6)*.1

    # elif laser_data.ranges[270] < 1.1 or laser_data.ranges[270]==float('inf'):
    else:
        linear_x = 0
        linear_y = -.2
        angular_z = 0.5

    # Set and publish velocity
    vel.linear.x = linear_x
    vel.linear.y = linear_y
    vel.angular.z = angular_z
    pub.publish(vel)

# Align bot with wall after getting close
def align_wall(laser_data):

    global isAligned

    # Velocity publisher
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)\

    vel = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    min_laser_dist = min(laser_data.ranges[180], laser_data.ranges[360], laser_data.ranges[540], laser_data.ranges[0])

    min_index = laser_data.ranges.index(min_laser_dist)

    # If bot is aligned, set flag
    if (laser_data.ranges[90] < 1.3 and laser_data.ranges[270] < 1.3) or (laser_data.ranges[90] < 1.3 and laser_data.ranges[270] < 1.3) or (laser_data.ranges[90] < 1.3 and laser_data.ranges[90] > 1.2 and laser_data.ranges[270] == float('inf')) or (laser_data.ranges[90] ==float('inf') and laser_data.ranges[270] < 1.25 and laser_data.ranges[270] > 1.2):
        print('done')
        angular_z = 0
        isAligned = True

    # Determine the direction in which the bot must turn...required only for corner cases
    elif not isAligned and laser_data.ranges[90] < 1.3 and laser_data.ranges[270] == float('inf'):
        print('right')
        angular_z = -.2
    elif not isAligned and laser_data.ranges[270] > 1.3 and laser_data.ranges[90] == float('inf'):
        print('right')
        angular_z = -.2

    elif not isAligned and laser_data.ranges[90] > 1.3 and laser_data.ranges[270] == float('inf'):
        print('left')
        angular_z = .2
    elif not isAligned and laser_data.ranges[270] < 1.3 and laser_data.ranges[90] == float('inf'):
        print('left')
        angular_z = .2

    # Set and publish velocity
    vel.linear.x = linear_x
    vel.linear.y = linear_y
    vel.angular.z = angular_z
    pub.publish(vel)

# Get close to the wall
def get_close(laser_data):

    global curr_x, curr_y, curr_z
    global getClose

    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)\

    vel = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0

    min_laser_dist = min(laser_data.ranges[90:270])

    min_index = laser_data.ranges.index(min_laser_dist)

    rot_angle = (min_index - 180)/2

    angular_z = rot_angle*.01

    if abs(rot_angle)<10:
        linear_x = min((min_laser_dist - .5)*.7,.5)
    else:
        linear_x = 0

    if abs(linear_x) < .05 and abs(angular_z) <.01:
        getClose = False

    vel.linear.x = linear_x
    vel.linear.y = linear_y
    vel.angular.z = angular_z
    pub.publish(vel)

if __name__ == "__main__":
    rospy.init_node('find_door')
    rospy.Subscriber('/scan',LaserScan,get_regions, queue_size=10)
    rospy.Subscriber('/odom',Odometry,get_pose,queue_size=10)
    rospy.spin()