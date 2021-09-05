#!/usr/bin/env python3

import rospy
import math
import actionlib
import matplotlib.pyplot as plt
import numpy as np
from numpy import where
from numpy import unique
import statistics

from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from scipy.cluster.vq import kmeans2
from sklearn.cluster import MeanShift
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf import transformations

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

global curr_x, curr_y, curr_z, wall_found

# Pose callback
def get_pose(data):

    global curr_x, curr_y, curr_z

    curr_x = data.pose.pose.position.x
    curr_y = data.pose.pose.position.y
    quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    curr_z = euler[2]

def movebase_client(goal_x, goal_y, quat):
    
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
   # Set frame id    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Set goal position
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
   # Set goal orientation 
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()

def laser_callback(data):

    pass

    # global curr_x, curr_y, curr_z

    # z = list(data.ranges)
    # for i in range(len(z)):
    #     if z[i]==float("inf"):
    #         z[i] = 0
    #     z[i] = list(pol2cart(z[i], math.radians(i/2)))

    # centroid,label = kmeans2(z, 4, minit='random')
    # clustering = MeanShift(bandwidth=2).fit(z)

    # clusters = unique(clustering)

    # x = [i[0] for i in z]
    # y = [i[1] for i in z]

    # plt.plot(x,y)

    # plt.show()

    # biggest_cluster = statistics.mode(label)

    # print(centroid[biggest_cluster])

    # plt.scatter(x,y, c=label, cmap='rainbow')
    # for cluster in clusters:
    # 	# get row indexes for samples with this cluster
    #     row_ix = where(clustering == cluster)
    #     # create scatter of these samples
    #     plt.scatter(x,y, c=clustering.labels_, cmap='rainbow')

    # plt.show()

def find_wall(laser_data):
    
    global wall_found
    global curr_x, curr_y, curr_z

    sorted_data = sorted(enumerate(laser_data.ranges), key=lambda x:x[1])
    for i in sorted_data:
        check_index = i[0]+2*int(math.degrees(math.atan2(1.5, i[1])))
        if check_index >=720:
            check_index = check_index - 720
        if abs(i[1] - laser_data.ranges[check_index]) < .2:
            print(check_index)
            wall_found = True
            gap = check_index - i[0]
            if gap < 0:
                gap = 720 + gap
            angle = math.radians(curr_z + ((i[0] - 180)/2) + (gap/4))
            mid = i[0]+gap
            if mid > 720:
                mid = mid -720
            quat_angle = quaternion_from_euler(0, 0, angle)
            if gap<180:
                result = movebase_client(curr_x + (laser_data.ranges[mid]-1)*abs(math.cos((gap-180)/2)) , curr_y - (laser_data.ranges[mid]-1)*abs(math.sin((gap-180)/2)), quat_angle)
            else:
                result = movebase_client(curr_x - (laser_data.ranges[mid]-1)*abs(math.cos((gap-180)/2)) , curr_y - (laser_data.ranges[mid]-1)*abs(math.sin((gap-180)/2)), quat_angle)
            
            exit()

if __name__ == "__main__":
    rospy.init_node('find_door')
    rospy.Subscriber('/scan', LaserScan, find_wall, queue_size=10)
    rospy.Subscriber('/odom',Odometry,get_pose,queue_size=10)
    rospy.spin()