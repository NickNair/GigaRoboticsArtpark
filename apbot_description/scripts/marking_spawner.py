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

    numTrash = input("Enter the number of markings (upto 5):\n")

    while True:
        try:
            numTrash = int(numTrash)
            break
        except:
            print("Invald input\n")

    cans = ['Black/10', 'Black/20', 'Black/30', 'Blue/10', 'Blue/20', 'Blue/30', 'Red/10', 'Red/20', 'Red/30']

    for i in range(numTrash):
        spawn_pose = Pose()

        can = random.choice(cans)

        cans.remove(can)

        model_path = "/artpark_workspace/src/GigaRoboticsArtpark_artpark/artpark2021_world/models/" + can
        model_xml = ''

        with open (model_path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        
        rand_x = random.uniform(-.8, .8)
        rand_y = random.uniform(-.8, .8)

        (spawn_pose.position.x,spawn_pose.position.y) = (rand_x, rand_y)

        spawn_pose.position.z = .055

        qaut_angle = quaternion_from_euler(0, 0, rand_x*rand_y)
        spawn_pose.orientation.x = qaut_angle[0]
        spawn_pose.orientation.y = qaut_angle[1]
        spawn_pose.orientation.z = qaut_angle[2]
        spawn_pose.orientation.w = qaut_angle[3]

        spawn_model_prox('marking'+str(i+1), model_xml, '', spawn_pose, 'world') 
