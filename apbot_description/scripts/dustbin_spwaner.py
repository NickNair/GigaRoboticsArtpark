#!/usr/bin/env python3

import rospy, tf
import rospkg
import math
import numpy as np
import random

from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_model", SpawnModel)
    
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 1

    spawn_pose = Pose()

    # Spawn the new model #
    model_path_green = "artpark_workspace/src/GigaRoboticsArtpark/artpark2021_world/models/dustbins/Dustbin_Green_Cylinder"
    model_path_yellow = "artpark_workspace/src/GigaRoboticsArtpark/artpark2021_world/models/dustbins/Dustbin_Black_Cylinder"
    model_xml_green = ''
    model_xml_yellow = ''

    with open (model_path_green + '/model.sdf', 'r') as xml_file:
        model_xml_green = xml_file.read().replace('\n', '')

    with open (model_path_yellow + '/model.sdf', 'r') as xml_file:
        model_xml_yellow = xml_file.read().replace('\n', '')

    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    numBins = input("Enter the number of dustbins (2-4):\n")
    while True:
        try:
            numBins = int(numBins)
            break
        except:
            print("Invald input\n")

    dustbin_poses = [(-1.3,0),(-.2,1.3),(1.3,.23),(-1.3,.85)]
    for i in range(numBins):
        qaut_angle = quaternion_from_euler(0, 0, 0)
        (spawn_pose.position.x,spawn_pose.position.y) = dustbin_poses[i]
        spawn_pose.position.z = .6
        spawn_pose.orientation.x = qaut_angle[0]
        spawn_pose.orientation.y = qaut_angle[1]
        spawn_pose.orientation.z = qaut_angle[2]
        spawn_pose.orientation.w = qaut_angle[3]
        if not i:
            spawn_model_prox('dustbin'+str(i+1), model_xml_green, '', spawn_pose, 'world') 
        spawn_model_prox('dustbin'+str(i+1), model_xml_yellow, '', spawn_pose, 'world')