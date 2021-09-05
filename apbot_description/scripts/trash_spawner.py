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
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    numCans = input("Enter the number of cans: (Max of 5)\n")
    while True:
        try:
            numCans = int(numCans)
            break
        except:
            print("Invald input\n")

    numCups = input("Enter the number of cups: (Max of 5)\n")
    while True:
        try:
            numCups = int(numCups)
            break
        except:
            print("Invald input\n")

    cans = ['pepsi_can', 'pepsi_can_crushed', 'coca_cola_can', 'coca_cola_can_crushed', 'fanta_can', 'fanta_can_crushed', 'fresca_can', 'fresca_can_crushed', 'mountain_dew_can', 'mountain_dew_can_crushed', 'sprite_can', 'sprite_can_crushed']
    cups = ['cup_blue', 'cup_brown', 'cup_green', 'cup_red', 'cup_white']

    for i in range(numCans):
        spawn_pose = Pose()

        can = random.choice(cans)

        cans.remove(can)

        can_model_path = "artpark_workspace/src/GigaRoboticsArtpark/artpark2021_world/models/cans/" + can
        model_xml = ''

        with open (can_model_path + '/model.sdf', 'r') as xml_file:
            can_model_xml = xml_file.read().replace('\n', '')
        
        rand_x = random.uniform(-.9, .9)
        rand_y = random.uniform(-.7, 1)

        (spawn_pose.position.x,spawn_pose.position.y) = (rand_x, rand_y)

        spawn_pose.position.z = .079

        qaut_angle = quaternion_from_euler(0, 1.57, rand_x*rand_y)
        spawn_pose.orientation.x = qaut_angle[0]
        spawn_pose.orientation.y = qaut_angle[1]
        spawn_pose.orientation.z = qaut_angle[2]
        spawn_pose.orientation.w = qaut_angle[3]

        spawn_model_prox('trash'+str(i+1), can_model_xml, '', spawn_pose, 'world') 

    for i in range(numCups):
        spawn_pose = Pose()

        cup = random.choice(cups)

        cups.remove(cup)

        cup_model_path = "artpark_workspace/src/GigaRoboticsArtpark/artpark2021_world/models/cups/" + cup
        model_xml = ''

        with open (cup_model_path + '/model.sdf', 'r') as xml_file:
            cup_model_xml = xml_file.read().replace('\n', '')
        
        rand_x = random.uniform(-1.2, 1.2)
        rand_y = random.uniform(-.7, 1.1)

        (spawn_pose.position.x,spawn_pose.position.y) = (rand_x, rand_y)

        spawn_pose.position.z = .079

        qaut_angle = quaternion_from_euler(0, 1.57, rand_x*rand_y)
        spawn_pose.orientation.x = qaut_angle[0]
        spawn_pose.orientation.y = qaut_angle[1]
        spawn_pose.orientation.z = qaut_angle[2]
        spawn_pose.orientation.w = qaut_angle[3]

        spawn_model_prox('trash'+str(numCans+i+1), cup_model_xml, '', spawn_pose, 'world') 