#! /usr/bin/env python3
import open3d
import numpy as np
from ctypes import * # convert float to uint32

import tf
import rospy
from std_msgs.msg import Header, String
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
from  visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2

global o3dpc

global header

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

def convertImage(msg):
    global received_ros_cloud
    received_ros_cloud = msg
    # rospy.loginfo("-- Received ROS PointCloud2 message.")

def convertCloudFromRosToOpen3d(ros_cloud):

    global header

    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
    
    header = ros_cloud.header.frame_id
    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="camera_depth_frame"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    fields=FIELDS_XYZ
    cloud_data=points
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def table_top_func(point_cloud,z_min,z_max, debug, open3d_debug):
    planes = []
    count = 0 
    check = False
    while(len(np.asarray(point_cloud.points)) > 1000):

        plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.0005, ransac_n=3, num_iterations=1000)
        inlier_cloud = point_cloud.select_by_index(inliers)

        if  debug or open3d_debug:
            print("cloud: ",count," Normals")
            [a, b, c, d] = plane_model
            print("a:",a)
            print("b:",b)
            print("c:",c)
            print("d:",d)
            print(plane_model)

        # Setting colour for planes
        if(count == 0):
            colours_red   = 0
            colours_green = 0
            colours_blue  = 1.0
        elif(count == 1):
            colours_red   = 0
            colours_green = 1.0
            colours_blue  = 1.0
        elif(count == 2):
            colours_red   = 0
            colours_green = 1.0
            colours_blue  = 0
        elif(count == 3):
            colours_red   = 1.0
            colours_green = 1.0
            colours_blue  = 1.0
        elif(count == 4):
            colours_red   = 1.0
            colours_green = 1.0
            colours_blue  = 0
        elif(count == 5):
            colours_red   = 1.0
            colours_green = 0
            colours_blue  = 0
        elif(count == 6):
            colours_red   = 1.0
            colours_green = 1.0
            colours_blue  = 0

        # setting the point clouds colour
        inlier_cloud.paint_uniform_color([colours_red, colours_green, colours_blue])

        # checking the normal of the plane 
        if (plane_model[0] < 0.05) and (plane_model[1] < 0.05) and (plane_model[2] > 0.95):
            
            # Finding the Center
            center = inlier_cloud.get_center()
            if (center[2] > z_min) and (center[2]<2.0):
                # Found the table
                planes.append(inlier_cloud)
                check = True
                return planes, check
        
        # Subtracting the previous plane
        point_cloud = point_cloud.select_by_index(inliers, invert=True)
        count+=1

    planes.append(point_cloud)

    return planes, check

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

def main():
    global received_ros_cloud
    global downsampled_cloud

    downsampled_cloud =[]
    
    rospy.init_node('plane_detection')

    # Subscribe to point cloud
    sub2 = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=convertImage, queue_size=10)

    # Topics publishing
    topic_debug = "apbot/points_debug"
    topic_table_top = "apbot/table_top"
    pub_debug = rospy.Publisher(topic_debug, PointCloud2, queue_size=1)
    pub_table_top = rospy.Publisher(topic_table_top, PointCloud2, queue_size=1)
    status_pub = rospy.Publisher('countertop_detected', String, queue_size=10)

    # The 2 variables that can be used to see the point clouds being processed 
    # Ros vizualization
    debug = 0 

    # open3d vizualization
    open3d_debug = 0

    # Transformation matix from camera frame to base_footprint to have a common frame to process the point cloud
    transform = transform_frames('/base_footprint', '/camera_depth_frame', debug, open3d_debug)
    
    # wait until we recive the point cloud, required for slow processors
    while True:
        try:
            received_ros_cloud
            break
        except:
            continue

    # Set the range in which the table will be
    z_min = 0.5
    z_max = 2.0

    check = False

    # Run the code untill stopped, it can be made into a service as well
    while not check:

        # Convert to open3d data type
        received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
        
        # Set the origin of the point cloud to be vizualized
        FOR = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0,0,0])        
        
        # After transformation
        received_open3d_cloud.transform(transform)

        if(debug):
            ros_plane = convertCloudFromOpen3dToRos(received_open3d_cloud, "base_footprint")
            pub_debug.publish(ros_plane)

        if(open3d_debug):
            open3d.visualization.draw_geometries([received_open3d_cloud, FOR])

        # The function to find the different planes
        print("Started Segmentation")
        found_plane, check = table_top_func(received_open3d_cloud, z_min, z_max, debug, open3d_debug)

        if(check):
            copy_of_plane=[found_plane[0]]
            center = found_plane[0].get_center()            
            print("Found plane at height: ",center[2])

            # Publishing the Table Top
            ros_plane_table_top = convertCloudFromOpen3dToRos(found_plane[0], "base_footprint")
            pub_table_top.publish(ros_plane_table_top)

            # Transform back to publish the table tf
            transform_back = transform_frames('/camera_depth_frame','/base_footprint', debug, open3d_debug)
            copy_of_plane[0].transform(transform_back)

            # Publishing the table tf 
            center_table_top = copy_of_plane[0].get_center()
            br = tf.TransformBroadcaster()
            br.sendTransform((center_table_top[0], center_table_top[1], center_table_top[2]), (0,0,0,1), rospy.Time.now(), "table_top", "camera_depth_frame")

            downsampled_cloud = [copy_of_plane[0].voxel_down_sample(voxel_size=0.05)]

            print(downsampled_cloud[0])
        else:
            print("Could not Find Table Top")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_table_top.publish(ros_plane_table_top)
        br = tf.TransformBroadcaster()
        br.sendTransform((center_table_top[0], center_table_top[1], center_table_top[2]), (0,0,0,1), rospy.Time.now(), "table_top", "camera_depth_frame")
        print("Published TF...")
        status_pub.publish("Countertop Detected")
        rate.sleep()


if __name__ == '__main__':
    main()