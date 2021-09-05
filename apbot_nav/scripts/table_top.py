#! /usr/bin/env python
from numpy import ma
from numpy.lib.function_base import append
import open3d
import numpy as np
from ctypes import * # convert float to uint32

import tf
import rospy
from std_msgs.msg import Header, String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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

class TableTop:

    def __init__(self):

        self.topic_debug = "apbot/points_debug"
        self.topic_table_top = "apbot/table_top"
        self.topic_sink_center = "apbot/sink_center"
        self.topic_table_top_center = "apbot/table_top_center"
        self.topic_table_dimensions = "apbot/table_dimensions"

        self.z_min = 0.5
        self.z_max = 2.0

        self.check = False

        self.publishing_frame = "base_footprint"

        self.global_frame = "base_footprint"

        self.table_top_center = PoseStamped()
        self.sink_center = PoseStamped()
        self.table_dimesions = PoseStamped()

        # The 2 variables that can be used to see the point clouds being processed 
        # Ros vizualization
        self.debug = 0 

        # open3d vizualization
        self.open3d_debug = 0

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

    def convertCloudFromOpen3dToRos(self,open3d_cloud, frame_id="camera_depth_frame"):
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

    def table_top_func(self,point_cloud,z_min,z_max, debug, open3d_debug):
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
                if (center[2] > z_min) and (center[2]<z_max):
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

        return homogenous,trans1,quat1

    def shortest_distance(self,x1, y1, a, b, c):
        
        d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
        
        return d

    def point_distance(self,point1,point2):
        d = ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 )**0.5
        
        return d

    def find_equation(self,point1,point2):
        a = point2[1] - point1[1]
        b = -(point2[0] - point1[0])
        c = point1[1]*(point2[0]-point1[0]) - point1[0]*(point2[1]-point1[1])

        return a,b,c

    def ros_publishers(self):
        self.pub_debug = rospy.Publisher(self.topic_debug, PointCloud2, queue_size=1)
        self.pub_table_top = rospy.Publisher(self.topic_table_top, PointCloud2, queue_size=1)
        self.pub_sink_center = rospy.Publisher(self.topic_sink_center, PoseStamped, queue_size=1)
        self.pub_table_top_center = rospy.Publisher(self.topic_table_top_center, PoseStamped, queue_size=1)
        self.pub_table_dimensions = rospy.Publisher(self.topic_table_dimensions , PoseStamped, queue_size=1)

    def ros_subscriber(self):
        # Subscribe to point cloud
        sub2 = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.convertImage, queue_size=10)

    def sink_clustering(self,table_points,debug):
        boundry_points = []
        x_t = [x[0] for x in table_points]
        y_t = [y[1] for y in table_points]

        x_min_i = np.argmin(x_t)
        x_min_p = table_points[x_min_i]
        y_min_i = np.argmin(y_t)
        y_min_p = table_points[y_min_i]
        x_max_i = np.argmax(x_t)
        x_max_p = table_points[x_max_i]
        y_max_i = np.argmax(y_t)
        y_max_p = table_points[y_max_i]         
        
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

        print("width", "length", width, length)

        self.table_dimesions.pose.position.x = width 
        self.table_dimesions.pose.position.y = length 

        table_center = [(x_min+x_max)/2,(y_min+y_max)/2]                   
        print(x_min)
        print(x_max)
        print(y_min)
        print(y_max)

        for i in range(0,len(table_points)):
            if abs(table_points[i][1] - table_points[i-1][1]) > 0.2:
                # if (abs(table_points[i-1][0] - x_min) > 0.2) and (abs(table_points[i-1][0] - x_max) > 0.2) and (abs(table_points[i-1][1] - y_max) > 0.003) and (abs(table_points[i-1][1] - y_min) > 0.003):
                #     boundry_points.append(table_points[i-1])  
                #     print("bounding points")  
                if (abs(table_points[i][0] - x_min) > 0.003) and (abs(table_points[i][0] - x_max) > 0.003) and (abs(table_points[i][1] - y_max) > 0.05) and (abs(table_points[i][1] - y_min) > 0.05):   
                    boundry_points.append(table_points[i-1])  
                    boundry_points.append(table_points[i])
                    print("bounding points")

        equation_1 = self.find_equation(x_min_p,y_max_p)
        equation_2 = self.find_equation(y_max_p,x_max_p)
        equation_3 = self.find_equation(x_max_p,y_min_p)
        equation_4 = self.find_equation(y_min_p,x_min_p)

        sink_points=[]
        for i in boundry_points:
            d1 = self.shortest_distance(i[0],i[1],equation_1[0],equation_1[1],equation_1[2])
            d2 = self.shortest_distance(i[0],i[1],equation_2[0],equation_2[1],equation_2[2])
            d3 = self.shortest_distance(i[0],i[1],equation_3[0],equation_3[1],equation_3[2])
            d4 = self.shortest_distance(i[0],i[1],equation_4[0],equation_4[1],equation_4[2])

            if not (d1<0.05 or d2<0.05 or d3<0.05 or d4<0.05 ):
                sink_points.append(i)
                print("Sink Point")
        
        self.sink_check = False
        if len(sink_points) > 4 :
            self.sink_check = True

        if(self.sink_check):
            # The following bandwidth can be automatically detected using
            bandwidth = estimate_bandwidth(sink_points, quantile=0.8, n_samples=70)
            ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
            ms.fit(sink_points)
            labels = ms.labels_
            cluster_centers = ms.cluster_centers_
            labels_unique = np.unique(labels)
            n_clusters_ = len(labels_unique)

            cluster_1 = []
            cluster_2 = []
            cluster_3 = []
            for i in range(len(sink_points)):
                for j in range(n_clusters_):
                    if labels[i] == labels_unique[j]:
                        cluster_1.append(sink_points[i])

            x_cl = [x[0] for x in cluster_1]
            y_cl = [y[1] for y in cluster_1] 

            sink_x_min_i = np.argmin(x_cl)
            sink_y_min_i = np.argmin(y_cl)
            sink_x_max_i = np.argmax(x_cl)
            sink_y_max_i = np.argmax(y_cl)

            sink_x_min_p = cluster_1[sink_x_min_i]
            sink_y_min_p = cluster_1[sink_y_min_i]
            sink_x_max_p = cluster_1[sink_x_max_i]
            sink_y_max_p = cluster_1[sink_y_max_i]

            sink_corner_points= [sink_x_min_p , sink_x_max_p , sink_y_min_p , sink_y_max_p]

        # Ploting the data
        x_b = [x[0] for x in boundry_points]
        y_b = [y[1] for y in boundry_points]

        x_c = [x[0] for x in corner_points]
        y_c = [y[1] for y in corner_points]           

        if(self.sink_check):       
            x_s = [x[0] for x in sink_points]
            y_s = [y[1] for y in sink_points] 

            x_cs = [x[0] for x in sink_corner_points]
            y_cs = [y[1] for y in sink_corner_points] 

        if debug:
            plt.scatter(x_t, y_t,s=32)
            plt.scatter(x_b,y_b,color='red', s=32)
            plt.scatter(x_c,y_c,color='green', s=32)

            if(self.sink_check):
                plt.scatter(x_s,y_s,color='k', s=32)
                plt.scatter(x_cs,y_cs,color='c', s=32)
            
                colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
                for k, col in zip(range(n_clusters_), colors):
                    my_members = labels == k
                    cluster_center = cluster_centers[k]
                    plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,markeredgecolor='k', markersize=14)

            plt.savefig('plot.png', dpi=300, bbox_inches='tight')
            plt.show()
        
        if not self.sink_check:
            cluster_centers = [-1,-1,-1]

        return cluster_centers,table_center

    def publishing_topics(self):

        # # Publishing the table top point cloud
        self.pub_table_top.publish(self.ros_plane_table_top)
        
        # Publishing the tf's
        br = tf.TransformBroadcaster()
        br.sendTransform((self.table_top_center.pose.position.x, self.table_top_center.pose.position.y, self.table_top_center.pose.position.z), (0,0,0,1), rospy.Time.now(), "table_top", self.global_frame)                
       
        if(self.sink_check):
            br = tf.TransformBroadcaster()
            br.sendTransform((self.sink_center.pose.position.x, self.sink_center.pose.position.y, self.sink_center.pose.position.z), (self.sink_center.pose.orientation.x, self.sink_center.pose.orientation.y, self.sink_center.pose.orientation.z, self.sink_center.pose.orientation.w), rospy.Time.now(), "sink", self.global_frame)                
        
            # Publishing the sink and the table poses
            self.sink_center.header.stamp = rospy.Time.now()
            self.pub_sink_center.publish(self.sink_center)

        self.table_top_center.header.stamp = rospy.Time.now()
        self.pub_table_top_center.publish(self.table_top_center)

        # Public table dimensions
        self.pub_table_dimensions.publish(self.table_dimesions)

        print("Published all Topics...")

        return self.ros_plane_table_top, self.table_top_center, self.sink_center

    def rotate_orientation(self, ori, q):
        rot_mat = tf.transformations.quaternion_matrix(q)
        pose_rot = rot_mat.dot([ori.x, ori.y, ori.z, ori.w])
        ori.x = pose_rot[0]
        ori.y = pose_rot[1]
        ori.z = pose_rot[2]
        ori.w = pose_rot[3]

    def translate_position(ori, pos, t):
        pos.x += t[0]
        pos.y += t[1]
        pos.z += t[2] 

    def start_detection(self):

        self.downsampled_cloud =[]
        
        # rospy.init_node('plane_detection')

        self.ros_subscriber()

        self.ros_publishers()

        # Transformation matix from camera frame to base_footprint to have a common frame to process the point cloud
        transform,_,_ = self.transform_frames('/base_footprint', '/camera_depth_frame', self.debug, self.open3d_debug)
        
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
            found_plane, self.check = self.table_top_func(received_open3d_cloud, self.z_min, self.z_max, self.debug, self.open3d_debug)

            if(self.check):
                copy_of_plane=[found_plane[0]]
                center = found_plane[0].get_center()            
                print("Found plane at height: ",center[2])

                table_points = np.asarray(found_plane[0].points)

                cluster_centers,table_center = self.sink_clustering(table_points,self.debug)

                if(self.sink_check):
                    # Publishing the sink center
                    self.sink_center.header.stamp = rospy.Time.now()
                    self.sink_center.header.frame_id = self.publishing_frame

                    self.sink_center.pose.position.x = cluster_centers[0][0]
                    self.sink_center.pose.position.y = cluster_centers[0][1]
                    self.sink_center.pose.position.z = center[2]

                    self.sink_center.pose.orientation.x = 0
                    self.sink_center.pose.orientation.y = 0
                    self.sink_center.pose.orientation.z = 0
                    self.sink_center.pose.orientation.w = 1.0

                    homogeous,translation,rotation = self.transform_frames(self.global_frame, self.publishing_frame, self.debug, self.open3d_debug)

                    self.rotate_orientation(self.sink_center.pose.orientation,rotation)
                    self.translate_position(self.sink_center.pose.position,translation)

                    self.sink_center.header.frame_id = self.global_frame
                    self.pub_sink_center.publish(self.sink_center)

                    # Sink tf
                    br = tf.TransformBroadcaster()
                    br.sendTransform((self.sink_center.pose.position.x, self.sink_center.pose.position.y, self.sink_center.pose.position.z), (self.sink_center.pose.orientation.x, self.sink_center.pose.orientation.y, self.sink_center.pose.orientation.z, self.sink_center.pose.orientation.w), rospy.Time.now(), "sink", self.global_frame)                

                # Publishing the Table Top
                self.ros_plane_table_top = self.convertCloudFromOpen3dToRos(found_plane[0], "base_footprint")
                self.pub_table_top.publish(self.ros_plane_table_top)

                # Publishing the table tf 
                center_table_top = copy_of_plane[0].get_center()

                # publishing the table top center         
                self.table_top_center.header.stamp = rospy.Time.now()
                self.table_top_center.header.frame_id = self.publishing_frame

                self.table_top_center.pose.position.x = table_center[0]
                self.table_top_center.pose.position.y = table_center[1]
                self.table_top_center.pose.position.z = center_table_top[2]

                self.table_top_center.pose.orientation.x = 0
                self.table_top_center.pose.orientation.y = 0
                self.table_top_center.pose.orientation.z = 0
                self.table_top_center.pose.orientation.w = 1.0

                homogeous,translation,rotation = self.transform_frames(self.global_frame, self.publishing_frame, self.debug, self.open3d_debug)

                self.rotate_orientation(self.table_top_center.pose.orientation,rotation)
                self.translate_position(self.table_top_center.pose.position,translation)

                br = tf.TransformBroadcaster()
                br.sendTransform((self.table_top_center.pose.position.x, self.table_top_center.pose.position.y, self.table_top_center.pose.position.z), (0,0,0,1), rospy.Time.now(), "table_top", self.global_frame)                

                self.table_dimesions.pose.position.z = self.table_top_center.pose.position.z
                self.pub_table_dimensions.publish(self.table_dimesions)

                self.table_top_center.header.frame_id = self.global_frame
                self.table_top_center.pose.position.z = 1
                quat_table = quaternion_from_euler(0,0,math.pi/2)
                self.table_top_center.pose.orientation.x = quat_table[0]
                self.table_top_center.pose.orientation.y = quat_table[1]
                self.table_top_center.pose.orientation.z = quat_table[2]
                self.table_top_center.pose.orientation.w = quat_table[3]

                self.pub_table_top_center.publish(self.table_top_center)

            else:
                print("Could not Find Table Top")
        
        table_dimensions  = [self.table_dimesions.pose.position.x, self.table_dimesions.pose.position.y, self.table_dimesions.pose.position.z]

        self.table_top_node_data = {"Table Center" : self.table_top_center, "Table Dimension" : table_dimensions, "Sink Center": self.sink_center, "Open 3D Table Top": found_plane[0]}

        return self.table_top_node_data
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     # Publishing the table top point cloud
        #     # self.pub_table_top.publish(self.ros_plane_table_top)
        #     self.publishing_topics()

        #     rate.sleep()


if __name__ == '__main__':
    table_top = TableTop()
    table_top.start_detection()