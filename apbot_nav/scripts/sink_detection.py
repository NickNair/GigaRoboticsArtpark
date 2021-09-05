#! /usr/bin/env python
from numpy.lib.function_base import append
import open3d
import numpy as np
from ctypes import * # convert float to uint32

import tf
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from  visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import math
from sklearn.cluster import MeanShift, estimate_bandwidth
from itertools import cycle

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

class SinkDetection:

    def __init__(self):

        self.topic_sink = "apbot/sink"
        self.topic_debug = "apbot/points_debug"
        self.topic_sink_center = "apbot/sink_center"
        self.topic_sink_dimensions = "apbot/sink_dimensions"

        self.downsampled_cloud =[]

        self.check = False

        # Set the range in which the table will be
        self.z_min = 0.5
        self.table_height = 1

        # Ros vizualization
        self.debug = 0 

        # open3d vizualization
        self.open3d_debug = 0

        self.global_frame = "odom"

        self.sink_center = PoseStamped()
        self.sink_top_dimensions = PoseStamped()
        self.sink_bottom_dimensions = PoseStamped()


    def convertImage(self,msg):
        # global received_ros_cloud
        self.received_ros_cloud = msg
        # rospy.loginfo("-- Received ROS PointCloud2 message.")

    def convertCloudFromRosToOpen3d(self,ros_cloud):

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

    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="camera_depth_frame"):
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

    def sink_func(self, point_cloud,z_min, table_height, debug, open3d_debug):
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
                if (center[2] > z_min) and (center[2]< table_height -0.1):
                    # Found the table
                    planes.append(inlier_cloud)
                    check = True
                    return planes, check
            
            # Subtracting the previous plane
            point_cloud = point_cloud.select_by_index(inliers, invert=True)
            count+=1

        planes.append(point_cloud)

        return planes, check

    def transform_frames(self,target_frame,source_frame, debug, open3d_debug):
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

    def point_distance(self,point1,point2):
        d = ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 )**0.5
        
        return d

    def points_below_table(self, point_cloud, sink_bottom, table_height, z_min):
        open3d_cloud = open3d.geometry.PointCloud()
        
        sink_dimensions = [0]* 4 # bottom_length, bottom width, top_length, top_width

        all_points = np.asarray(point_cloud.points)

        sink_bottom_points = np.asarray(sink_bottom.points)

        sink_center = sink_bottom.get_center()

        x_s = [x[0] for x in sink_bottom_points]
        y_s = [y[1] for y in sink_bottom_points]

        x_min_i = np.argmin(x_s)
        x_min_p = sink_bottom_points[x_min_i]
        y_min_i = np.argmin(y_s)
        y_min_p = sink_bottom_points[y_min_i]
        x_max_i = np.argmax(x_s)
        x_max_p = sink_bottom_points[x_max_i]
        y_max_i = np.argmax(y_s)
        y_max_p = sink_bottom_points[y_max_i]         
        
        x_min = min(x_s)
        x_max = max(x_s)
        y_min = min(y_s)
        y_max = max(y_s)  

        corner_points= [x_min_p , x_max_p , y_min_p , y_max_p]
        
        if abs(x_min_p[1] - y_max_p[1]) < 0.1:
            width1  = self.point_distance(x_min_p,y_max_p)
            length1 = self.point_distance(y_max_p,x_max_p)
            width2  = self.point_distance(x_max_p,y_min_p)
            length2 = self.point_distance(y_min_p,x_min_p)
            
            if abs(width2 - width1) > 0.05:
                width  = x_max - x_min
                length = y_max - y_min 
            else:
                width  = (width1 + width2)/2
                length = (length2 + length1)/2

        else:
            length1 = self.point_distance(x_min_p,y_max_p)
            width1  = self.point_distance(y_max_p,x_max_p)
            length2 = self.point_distance(x_max_p,y_min_p)
            width2  = self.point_distance(y_min_p,x_min_p)

            if abs(width2 - width1) > 0.05:
                width  = x_max - x_min
                length = y_max - y_min
            else:
                width  = (width1 + width2)/2
                length = (length2 + length1)/2

        print("width bottom", "length bottom", width, length)

        sink_dimensions[0] = width
        sink_dimensions[1] = length

        radius = max((table_height-sink_center[2]), (x_max-x_min)/2, (y_max-y_min)/2)
        below_table_points = []
        for i in all_points:
            if ((i[2] < table_height and z_min < i[2]) and ( ((i[0]-sink_center[0])**2 + (i[1]-sink_center[1])**2 + (i[2]-table_height)**2)**0.5 < (radius+0.05))):
                below_table_points.append(i)

        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(below_table_points))
        open3d_cloud.paint_uniform_color([0,1.0,1.0])

        x_t = [x[0] for x in below_table_points]
        y_t = [y[1] for y in below_table_points]

        x_t = [x[0] for x in below_table_points]
        y_t = [y[1] for y in below_table_points]

        x_min_i = np.argmin(x_t)
        x_min_p = below_table_points[x_min_i]
        y_min_i = np.argmin(y_t)
        y_min_p = below_table_points[y_min_i]
        x_max_i = np.argmax(x_t)
        x_max_p = below_table_points[x_max_i]
        y_max_i = np.argmax(y_t)
        y_max_p = below_table_points[y_max_i]         
        
        x_min = min(x_t)
        x_max = max(x_t)
        y_min = min(y_t)
        y_max = max(y_t)  

        corner_points= [x_min_p , x_max_p , y_min_p , y_max_p]
        
        if abs(x_min_p[1] - y_max_p[1]) < 0.1:
            width1  = self.point_distance(x_min_p,y_max_p)
            length1 = self.point_distance(y_max_p,x_max_p)
            width2  = self.point_distance(x_max_p,y_min_p)
            length2 = self.point_distance(y_min_p,x_min_p)
            
            if abs(width2 - width1) > 0.05:
                width  = x_max - x_min
                length = y_max - y_min 
            else:
                width  = (width1 + width2)/2
                length = (length2 + length1)/2

        else:
            length1 = self.point_distance(x_min_p,y_max_p)
            width1  = self.point_distance(y_max_p,x_max_p)
            length2 = self.point_distance(x_max_p,y_min_p)
            width2  = self.point_distance(y_min_p,x_min_p)

            if abs(width2 - width1) > 0.05:
                width  = x_max - x_min
                length = y_max - y_min
            else:
                width  = (width1 + width2)/2
                length = (length2 + length1)/2

        print("width top sink", "length top sink", width, length)

        x_c = [x[0] for x in corner_points]
        y_c = [y[1] for y in corner_points] 

        sink_dimensions[2] = width 
        sink_dimensions[3] = length         
        # plt.scatter(x_t, y_t,s=32)
        # plt.scatter(x_c,y_c,color='green', s=64)
        # plt.show()

        return open3d_cloud, sink_dimensions

    def ros_publishers(self):
        self.pub_debug = rospy.Publisher(self.topic_debug, PointCloud2, queue_size=1)
        self.pub_sink = rospy.Publisher(self.topic_sink, PointCloud2, queue_size=1)

    def ros_subscribers(self):
        # Subscribe to point cloud
        sub_points = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.convertImage, queue_size=10)
        # sub_table_height = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.convertImage, queue_size=10)
    
    def publishing_topics(self):
            self.pub_sink.publish(self.ros_plane_sink)
            br = tf.TransformBroadcaster()
            br.sendTransform((self.center_sink[0], self.center_sink[1], self.center_sink[2]), (0,0,0,1), rospy.Time.now(), "sink", self.global_frame)      

    def start_sink_detection(self):
        
        # rospy.init_node('sink_detection')

        self.ros_publishers()

        self.ros_subscribers()

        # Transformation matix from camera frame to base_footprint to have a common frame to process the point cloud
        transform = self.transform_frames('/base_footprint', '/camera_depth_frame', self.debug, self.open3d_debug)
        
        # wait until we recive the point cloud, required for slow processors
        while True:
            try:
                self.received_ros_cloud
                break
            except:
                continue

        # Run the code untill stopped, it can be made into a service as well
        while not self.check:

            # Convert to open3d data type
            received_open3d_cloud = self.convertCloudFromRosToOpen3d(self.received_ros_cloud)
            
            # Set the origin of the point cloud to be vizualized
            FOR = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0,0,0])        
            
            # After transformation
            received_open3d_cloud.transform(transform)

            if(self.debug):
                ros_plane = self.convertCloudFromOpen3dToRos(received_open3d_cloud, "base_footprint")
                self.pub_debug.publish(ros_plane)

            if(self.open3d_debug):
                open3d.visualization.draw_geometries([received_open3d_cloud, FOR])

            # The function to find the different planes
            print("Started Segmentation")
            found_plane, self.check = self.sink_func(received_open3d_cloud, self.z_min, self.table_height, self.debug, self.open3d_debug)

            if(self.check):
                copy_of_plane=[found_plane[0]]
                center = found_plane[0].get_center()            
                print("Found sink plane at height: ",center[2])

                self.pc_below_sink, self.sink_dimensions = self.points_below_table(received_open3d_cloud, found_plane[0], self.table_height, self.z_min)


                # Publishing the Sink
                self.ros_plane_sink = self.convertCloudFromOpen3dToRos(self.pc_below_sink, "base_footprint")
                self.pub_sink.publish(self.ros_plane_sink)

                # publishing the Sink center         
                self.sink_center.header.stamp = rospy.Time.now()
                self.sink_center.header.frame_id = "base_footprint"

                self.sink_center.pose.position.x = center[0]
                self.sink_center.pose.position.y = center[1]
                self.sink_center.pose.position.z = center[2]

                self.sink_center.pose.orientation.x = 0
                self.sink_center.pose.orientation.y = 0
                self.sink_center.pose.orientation.z = 0
                self.sink_center.pose.orientation.w = 1.0

                # publishing the Sink top dimentions         
                self.sink_top_dimensions.header.stamp = rospy.Time.now()
                self.sink_top_dimensions.header.frame_id = "base_footprint"

                self.sink_top_dimensions.pose.position.x = self.sink_dimensions[0]
                self.sink_top_dimensions.pose.position.y = self.sink_dimensions[1]
                self.sink_top_dimensions.pose.position.z = self.table_height

                # publishing the Sink bottom dimentions         
                self.sink_bottom_dimensions.header.stamp = rospy.Time.now()
                self.sink_bottom_dimensions.header.frame_id = "base_footprint"

                self.sink_bottom_dimensions.pose.position.x = self.sink_dimensions[2]
                self.sink_bottom_dimensions.pose.position.y = self.sink_dimensions[3]
                self.sink_bottom_dimensions.pose.position.z = center[2]
                # # Transform back to publish the table tf
                # transform_back = self.transform_frames(self.global_frame,'/base_footprint', self.debug, self.open3d_debug)
                # copy_of_plane[0].transform(transform_back)

                # # Publishing the table tf 
                # self.center_sink = copy_of_plane[0].get_center()
                # br = tf.TransformBroadcaster()
                # br.sendTransform((self.center_sink[0], self.center_sink[1], self.center_sink[2]), (0,0,0,1), rospy.Time.now(), "sink", self.global_frame)

            else:
                print("Could not Find Sink")
                return False
        
        self.sink_bottom = [self.sink_dimensions[2], self.sink_dimensions[3], center[2]]

        self.sink_top = [self.sink_dimensions[0], self.sink_dimensions[1], self.table_height]

        self.sink_node_data = {"Sink Center": center, "Sink Top Dimensions": self.sink_top, "Sink Bottom Dimensions" : self.sink_bottom, "Open 3D Sink": self.pc_below_sink}

        return self.sink_node_data

        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.publishing_topics()
        #     print("Published TF...")
        #     rate.sleep()


if __name__ == '__main__':
    sink_det = SinkDetection()
    sink_det.start_sink_detection()