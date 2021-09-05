#!/usr/bin/env python3

import tf
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs
from uuv_plume_model import Plume
from uuv_plume_msgs.srv import *
from sensor_msgs.msg import PointCloud
import numpy as np

global wet_points 

def transform_frames(target_frame,source_frame, debug, open3d_debug):
    #Finding the transform
    listner = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            trans1, quat1 = listner.lookupTransform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    homogenous = []
    homogenous = tf.transformations.quaternion_matrix(quat1)

    for i in range(0,3):
	    homogenous[i][3] = trans1[i]

    if debug or open3d_debug:
        print("matrix")
        print(homogenous)

    return homogenous

def transform_pose(input_pose, from_frame, to_frame):
    
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def callback(ros_point_cloud):
    global table_height
    global wet_points
    wet_points = []
    pose = Pose()
    from_frame = 'gripper_base_link'
    to_frame = 'base_footprint'
    for point in ros_point_cloud.points:
        pose.position.x = point.x
        pose.position.y = point.y
        pose.position.z = point.z
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        pose = transform_pose(pose,from_frame,to_frame)
        if(pose.position.z < table_height ):
            wet_points = np.append(pose)


class coefficients:
    x = 0
    y = 0
    z = 0

def main():
    rospy.init_node("spray")
    
    while True:
        try:
            print("waiting")
            rospy.wait_for_service('/plume//create_passive_scalar_turbulent_plume')
            print("waiting part 2")
            rospy.wait_for_service('/current_velocity_server/set_current_velocity')
            break
        except:
            continue

    # Creating a connection for the service 

    particle_service  = rospy.ServiceProxy('/plume/create_passive_scalar_turbulent_plume', CreatePassiveScalarTurbulentPlume)
    veclocity_service = rospy.ServiceProxy('/current_velocity_server/set_current_velocity',SetCurrentVelocity)

    #cloud_sub = rospy.Subscriber("/plume/particles", PointCloud, callback,queue_size=1,buff_size=52428800)    

    global table_height 
    table_height = 1

    plume_particles(particle_service,veclocity_service)

    #print(cloud_sub)

    particle_caller()


def particle_caller():
    rospy.spin()

def plume_particles(particle_service,veclocity_service):
    # turbulent_diffusion_coefficients.x = {'x': 0.0, 'y': 0.01, 'z': 0.08}
    turbulent_diffusion_coefficients = coefficients()
    source = coefficients()

    turbulent_diffusion_coefficients.x = 0.00003
    turbulent_diffusion_coefficients.y = 0.00003
    turbulent_diffusion_coefficients.z = 0.00003

    source.x = 0.0
    source.y = -0.0
    source.z = -0.0

    buoyancy_flux = 0.05
    stability_param = 0.001
    n_points = 1000000
    max_particles_per_iter = 10
    x_min = -1.0
    x_max = 1.0
    y_min = -1.0
    y_max = 1.0
    z_min = -1.0
    z_max = 1.0
    max_life_time = -1

    particle_resp = particle_service(turbulent_diffusion_coefficients, source , buoyancy_flux, stability_param, n_points, max_particles_per_iter, x_min, x_max, y_min, y_max, z_min, z_max, max_life_time)

    vel = -0.1
    hor_ang = 90.0
    velocity_resp = veclocity_service( vel, hor_ang)

    print(velocity_resp)
    print(particle_resp,"tt")


if __name__ == '__main__':
    main()

