#! /usr/bin/env python3
import rospy
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon,PoseStamped
import numpy as np
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import PointCloud,PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf
import threading
# import pcl

from uuv_plume_model import Plume
from uuv_plume_msgs.srv import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# rospy.init_node('node_2', anonymous=True)
# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)

class coefficients:
    x = 0
    y = 0
    z = 0

class MarkerSpawner:
    def __init__(self):
        self.marker2 = MarkerArray()
        self.flag = -1
        self.count=0
        self.a=[]
        self.isDelete = False
        # rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback2, queue_size=1000)
        
        self.pub_rviz_marker = rospy.Publisher('water_marker', MarkerArray, queue_size=10.,latch= True)

    def register_plume(self):
        self.plume_sub = rospy.Subscriber("/plume/particles", PointCloud, self.callback,queue_size=1000, tcp_nodelay=True)

    def unregister_plume(self):
        self.plume_sub.unregister()
	
    def spawn_marker(self,x,y,z):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.action = Marker().ADD
        self.marker.type = Marker().CYLINDER
        self.marker.lifetime.secs = -1
        self.marker.header.stamp = rospy.Time.now()
        self.cl= ColorRGBA(0,1,1,1)
        self.marker.color = self.cl
        self.marker.ns= "df "+str(self.count)
        self.count +=1
        pose_0 = PoseStamped()
        pose_0.header.frame_id= "map"
        pose_0.pose.position.x = x
        pose_0.pose.position.y = y
        pose_0.pose.position.z = z
        q=[0,0,1,0]
        pose_0.pose.orientation.x = q[0]
        pose_0.pose.orientation.y = q[1]
        pose_0.pose.orientation.z = q[2]
        pose_0.pose.orientation.w = q[3]
        self.marker.pose = pose_0.pose
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.001
        self.marker2.markers.append(self.marker)
        np.random.shuffle(self.marker2.markers)

    def ret_x_y(self, pose):
        return [pose.position.x ,pose.position.y]

    def delelemarkers(self):
        print("entering del")
        listener = tf.TransformListener()
        while True:
            try:
                t, =listener.lookupTransform("/map","/gripper_base_link", rospy.Time(0))
                break
            except:
                continue
        box_size = .15

        polygon = Polygon([(t[0] - box_size, t[1] -box_size), (t[0] - box_size, t[1] + box_size ), (t[0] + box_size, t[1] -box_size), (t[0]+ box_size, t[1] + box_size)])

        for i in self.marker2.markers:
            point = self.ret_x_y(i.pose)
            if polygon.contains(Point(point[0], point[1])):
                self.marker2.markers.revove(i)
        self.pub_rviz_marker.publish(self.marker2)
    
    def same_point(self,x,y,z,point1):
        if  abs(z-point1.z)<0.05 and abs(x-point1.x)<0.05 and abs(y-point1.y)<0.05: 
            return True
        return False

    def thread3(self,point,tes):
        pose_0 = PoseStamped()
        pose_0.header.frame_id= "gripper_base_link"
        pose_0.pose.position.x = point.x
        pose_0.pose.position.y = point.y
        pose_0.pose.position.z = point.z
        while True:
            if tes.frameExists("/map"):
                print("map is ok")
            try :
                p_wrt_odom = tes.transformPose("/map", pose_0)
                break
            except:
                continue
        if p_wrt_odom.pose.position.z < 1.3 and p_wrt_odom.pose.position.z > 0.8 :
            print("Spawn!!")
            self.spawn_marker(p_wrt_odom.pose.position.x,p_wrt_odom.pose.position.y,1)
            self.pub_rviz_marker.publish(self.marker2)

    def thread2(self,x,y,z,point):
        if self.same_point(x,y,z,point):
            print("Spawn!!")
            self.spawn_marker(x,y-0.01,z)
        
    def thread_function(self,point,cloud_data):
        np.random.shuffle(cloud_data)
        a = self.a 
        count=0
        threads = list()

        for x,y,z,rgb in cloud_data[::4000]:
            xt = threading.Thread(target=self.thread2, args=(x,y,z,point))
            threads.append(xt)
            xt.start() 
            print(count)
            count+=1
        for index, thread in enumerate(threads):
            thread.join() 
    
    def callback2(self,msg):
        if True:
            try:
                trans = tf_buffer.lookup_transform('gripper_base_link', msg.header.frame_id,
                                            msg.header.stamp)
            except tf2.LookupException as ex:
                rospy.logwarn(ex)
                return
            except tf2.ExtrapolationException as ex:
                rospy.logwarn(ex)
                return
            self.ros_point_cloud = do_transform_cloud(msg, trans) 
                

    def spray_particle(self):
        field_names=[field.name for field in self.ros_point_cloud.fields]
        cloud_data = list(pc2.read_points(self.ros_point_cloud, skip_nans=True, field_names = field_names))
        self.marker2.markers[:5] =[]
        count=0
        #print("le",len(self.marker2.markers))
        #print("length of point cloud",len(cloud_data))
        threads = list()
        print("Number of points in Point cloud: ",len(cloud_data))
        print("Number of points in Plume cloud: ",len(self.plume_point_cloud.points))
        for point in self.plume_point_cloud.points[::40]:
            x = threading.Thread(target=self.thread_function, args=(point,cloud_data))
            threads.append(x)
            x.start()   
        for index, thread in enumerate(threads):
            thread.join()         
        print("out") 
        self.pub_rviz_marker.publish(self.marker2)  
    
    def callback(self, data):
        print("entering")
        
        if self.isDelete:
            self.delelemarkers()
        else:
            print("entering2")
            tes = tf.TransformListener()
            self.plume_point_cloud = data 
            threads = list()
            for point in self.plume_point_cloud.points[::10]:
                pose_0 = PoseStamped()
                pose_0.header.frame_id= "gripper_base_link"
                pose_0.pose.position.x = point.x
                pose_0.pose.position.y = point.y
                pose_0.pose.position.z = point.z
                while True:
                    if tes.frameExists("/map"):
                        print("map is ok")
                    try :
                        p_wrt_odom = tes.transformPose("/map", pose_0)
                        break
                    except:
                        continue
                if p_wrt_odom.pose.position.z < 1.1 and p_wrt_odom.pose.position.z > 1:
                    print("Spawn!!")
                    # x_noise = np.random.normal(0, 0.03, 50)
                    # y_noise = np.random.normal(0, 0.03, 50)
                    # for i in range(50):
                    #     self.spawn_marker(p_wrt_odom.pose.position.x + x_noise[i],p_wrt_odom.pose.position.y + y_noise[i] ,1)
                    self.spawn_marker(p_wrt_odom.pose.position.x,p_wrt_odom.pose.position.y,1.05)
                    self.pub_rviz_marker.publish(self.marker2)
                    break

    def start_spray(self):
        while True:
            try:
                rospy.wait_for_service('/plume//create_passive_scalar_turbulent_plume')
                rospy.wait_for_service('/current_velocity_server/set_current_velocity')
                break
            except:
                continue

        particle_service  = rospy.ServiceProxy('/plume/create_passive_scalar_turbulent_plume', CreatePassiveScalarTurbulentPlume)
        veclocity_service = rospy.ServiceProxy('/current_velocity_server/set_current_velocity',SetCurrentVelocity)
   
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
        hor_ang = 99.0
        velocity_resp = veclocity_service( vel, hor_ang)

    def stop_spray(self):
        while True:
            try:
                rospy.wait_for_service('/plume//create_passive_scalar_turbulent_plume')
                rospy.wait_for_service('/current_velocity_server/set_current_velocity')
                break
            except:
                continue

        particle_service  = rospy.ServiceProxy('/plume/create_passive_scalar_turbulent_plume', CreatePassiveScalarTurbulentPlume)
        veclocity_service = rospy.ServiceProxy('/current_velocity_server/set_current_velocity',SetCurrentVelocity)
   
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
        n_points = 0
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
        hor_ang = 99.0
        velocity_resp = veclocity_service( vel, hor_ang)



