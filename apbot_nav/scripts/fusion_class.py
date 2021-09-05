#!/usr/bin/env python3

import rospy
import numpy as np
import time
import math

from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf
from std_msgs.msg import String,Int64MultiArray
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from vision_msgs.msg import Detection2DArray
from multiprocessing import Process, Lock

mutex = Lock()

class fusion:
    def __init__(self, debug=""):
        self.labels =[]
        self.labels_ = []
        self.L = []
        self.L_ = []
        self.count = 0
        self.count_ = 0
        self.final_labels = []
        self.final_labels_ = []
        self.detect_time =0 
        self.debug = debug
        self.bridge = CvBridge()
        self.boundingboxes_d = Detection2DArray()
    
    def callback(self,data):
        self.check = data.data
    
    def objects_callback(self,data):
        self.boundingboxes1 = data

    def objects_callback_d(self,data):
        self.boundingboxes_d = data
    
    def image_info_callback(self,data):
        self.K = 1/data.K[4]
    
    
    def same_object(self,Object):
        count = 0
        for i in self.final_labels:
            if abs(Object[0].pose.position.x-i[0].pose.position.x)<=0.05 and abs(Object[0].pose.position.y-i[0].pose.position.y)<=0.05:
                return [False,count,i]
            count+=1
        return [True]

    def same_object_(self,Object):
        count = 0
        for i in self.final_labels_:
            if abs(Object[0].pose.position.x-i[0].pose.position.x)<=0.05 and abs(Object[0].pose.position.y-i[0].pose.position.y)<=0.05:
                return [False,count,i]
            count+=1
        return [True]    

    def get_depth(self,img,x,y,cx,cy,fx,fy):
        try:
            center_x , center_y = cx,cy
            unit_scaling = 1
            constant_x = unit_scaling/fx
            constant_y = unit_scaling/fy
            depth = img[int(y)][int(x)]
            return [ (x - center_x)*depth*constant_x ,(y - center_y)*depth*constant_y , depth*unit_scaling]
        except:
            # print("ERROOOOOOR")
            return False 

    def depth_image_callback2(self, data):
        self.image_data = data

    def depth_image_callback(self,data):
        print("enter callback:....",self.debug)
        self.boundingboxes = self.boundingboxes1
        if(self.check=="Detect"):
            try:
                img = self.bridge.imgmsg_to_cv2(data, "32FC1")
            except CvBridgeError as e:
                print(e)
            K = self.K
            dets = len(self.boundingboxes.detections)
            if(len(self.boundingboxes.detections)):
                
                for i in range(len(self.boundingboxes.detections)):
                    id = i 
                    totalSampledX,totalSampledY,totalSampledCenter=0,0,0
                    box_size_x,box_size_y = self.boundingboxes.detections[i].bbox.size_x,self.boundingboxes.detections[i].bbox.size_y

                    center = Point()
                    xAxis = Point()
                    yAxis = Point()
                    center.x = self.boundingboxes.detections[i].bbox.center.x
                    center.y = self.boundingboxes.detections[i].bbox.center.y 
                    if self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                        center3D = self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                    else:
                        continue
                    brk = 1 
                    for j in range(int(box_size_x)):
                        for k in range(4):
                            xAxis.x, xAxis.y = center.x + j ,center.y + k
                            if self.get_depth(img,xAxis.x  , xAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                                axisSampledX = self.get_depth(img,xAxis.x  , xAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            else:
                                brk = 0 
                                break
                            yAxis.x, yAxis.y = center.x+ k, center.y - j
                            if self.get_depth(img,yAxis.x  , yAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                                axisSampledY = self.get_depth(img,yAxis.x  , yAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            else:
                                brk =0
                                break
                            axisMeanX =[0,0,0]
                            axisMeanX[0] += axisSampledX[0]
                            axisMeanX[1] += axisSampledX[1]
                            axisMeanX[2] += axisSampledX[2]
                            totalSampledX+=1

                            axisMeanY =[0,0,0]
                            axisMeanY[0] += axisSampledY[0]
                            axisMeanY[1] += axisSampledY[1]
                            axisMeanY[2] += axisSampledY[2]
                            totalSampledY+=1
                        if brk ==0 :
                            break
                    if brk ==0:
                        continue
                    for i in range(len(axisMeanX)):
                        axisMeanX[i] = axisMeanX[i]/totalSampledX
                        axisMeanY[i] = axisMeanY[i]/totalSampledY
                    
                    
                    n_xAxis = Vector3(axisMeanX[0] - center3D[0], axisMeanX[1] - center3D[1], axisMeanX[2] - center3D[2])
                    n_yAxis = Vector3(axisMeanY[0] - center3D[0], axisMeanY[1] - center3D[1], axisMeanY[2] - center3D[2])
                    n_zAxis = np.cross([axisMeanX[0] - center3D[0], axisMeanX[1] - center3D[1], 
                    axisMeanX[2] - center3D[2]],[axisMeanY[0] - center3D[0], axisMeanY[1] - center3D[1], axisMeanY[2] - center3D[2]])
                    n_zAxis =Vector3(n_zAxis[0],n_zAxis[1],n_zAxis[2])
                    M =[ [n_xAxis.x, n_yAxis.x,n_zAxis.x,0],
                        [n_xAxis.y, n_yAxis.y, n_zAxis.y,0],
                        [n_xAxis.z, n_yAxis.z, n_zAxis.z,0],
                        [  0, 0, 0, 1 ]]
                    for a in range(3):
                        for b in range(3):
                            M[a][b] = M[a][b]/(max(M[0][a],M[1][a],M[2][a])- min(M[0][a],M[1][a],M[2][a]))
                    M = np.asarray(M)

                    q= tf.transformations.quaternion_from_matrix(M)
                    q*=tf.transformations.quaternion_from_euler(0, -3.14/2,-3.14/2)
                    q = q/(sum(q**2)**0.5 )
                    q = [0,0,1,0]
                    
                    pose_0 = PoseStamped()
                    pose_0.header.frame_id= "camera_depth_frame"
                    pose_0.pose.position.x = center3D[0]
                    pose_0.pose.position.y = center3D[1]
                    pose_0.pose.position.z = center3D[2]

                    pose_0.pose.orientation.x = q[0]
                    pose_0.pose.orientation.y = q[1]
                    pose_0.pose.orientation.z = q[2]
                    pose_0.pose.orientation.w = q[3]
                    
                    tes = tf.TransformListener()
                    while True:
                        try :
                            p_wrt_odom = tes.transformPose("/map", pose_0)
                            break
                        except:
                            continue
                    if self.boundingboxes.detections[id].results[0].id==0 or self.boundingboxes.detections[id].results[0].id==1:
                        if "object_"+str(id+self.count) not in self.labels:  
                            self.L.append([p_wrt_odom,id +self.count])
                            self.labels.append("object_"+str(id+self.count))
                    elif self.boundingboxes.detections[id].results[0].id==2:
                        if "object_m_"+str(id+self.count) not in self.labels_: 
                            self.L_.append([p_wrt_odom,id +self.count])
                            self.labels_.append("object_m_"+str(id+self.count))
            flag =1
            # print("Detection:",self.detect_time)

            # if self.detect_time == 20:
            #     self.detect_time = 0
            #     pub = rospy.Publisher("/check", String,  queue_size=1, latch=True)

            #     flag_off = String()
                
            #     flag_off.data = "stop"
                
            #     pub.publish(flag_off)
            #     flag=0
                #detect_time = 0 

            # self.detect_time +=flag        
                
        else:
            # print("Else of Depth Image Callback",self.count)
            self.count = len(self.labels) + len(self.labels_)
            pub = rospy.Publisher('id_array', Int64MultiArray, queue_size=10,latch=True)
            ar = Int64MultiArray()
            # print(self.labels)
            # print(self.final_labels)
            for i in self.L:
                if self.same_object(i)[0]:
                    self.final_labels.append(i)
                else:
                    self.final_labels[self.same_object(i)[1]] = self.same_object(i)[2]
            # print([str(round(i[0].pose.position.x,2))+"_"+str(round(i[0].pose.position.y,2)) for i in self.final_labels])
            for i in self.final_labels:
                br = tf.TransformBroadcaster()
                br.sendTransform([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],
                                [i[0].pose.orientation.x,i[0].pose.orientation.y,i[0].pose.orientation.z,i[0].pose.orientation.w],
                                rospy.Time.now(),
                                "object_"+str(i[1]),
                                "map")
                ar.data.append(i[1])
            pub.publish(ar)

            # print("Else of Depth Image Callback",self.count)
            self.count_ = len(self.labels_)
            pub = rospy.Publisher('id_array_', Int64MultiArray, queue_size=10,latch=True)
            ar = Int64MultiArray()
            # print(self.labels)
            # print(self.final_labels)
            for i in self.L_:
                if self.same_object_(i)[0]:
                    self.final_labels_.append(i)
                else:
                    self.final_labels_[self.same_object_(i)[1]] = self.same_object_(i)[2]
            # print([str(round(i[0].pose.position.x,2))+"_"+str(round(i[0].pose.position.y,2)) for i in self.final_labels])
            for i in self.final_labels_:
                br = tf.TransformBroadcaster()
                br.sendTransform([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],
                                [i[0].pose.orientation.x,i[0].pose.orientation.y,i[0].pose.orientation.z,i[0].pose.orientation.w],
                                rospy.Time.now(),
                                "object_m_"+str(i[1]),
                                "map")
                ar.data.append(i[1])
            pub.publish(ar)

    def depth_image_callback_marking(self,data):
        print("enter callback:....",self.debug)
        self.boundingboxes = self.boundingboxes1
        if(self.check=="Detect"):
            try:
                img = self.bridge.imgmsg_to_cv2(data, "32FC1")
            except CvBridgeError as e:
                print(e)
            K = self.K
            dets = len(self.boundingboxes.detections)
            if(len(self.boundingboxes.detections)):
                
                for i in range(len(self.boundingboxes.detections)):
                    id = i 
                    totalSampledX,totalSampledY,totalSampledCenter=0,0,0
                    box_size_x,box_size_y = self.boundingboxes.detections[i].bbox.size_x,self.boundingboxes.detections[i].bbox.size_y

                    center = Point()
                    xAxis = Point()
                    yAxis = Point()
                    center.x = self.boundingboxes.detections[i].bbox.center.x
                    center.y = self.boundingboxes.detections[i].bbox.center.y 
                    if self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                        center3D = self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                    else:
                        continue
                    brk = 1 
                    for j in range(int(box_size_x)):
                        for k in range(4):
                            xAxis.x, xAxis.y = center.x + j ,center.y + k
                            if self.get_depth(img,xAxis.x  , xAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                                axisSampledX = self.get_depth(img,xAxis.x  , xAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            else:
                                brk = 0 
                                break
                            yAxis.x, yAxis.y = center.x+ k, center.y - j
                            if self.get_depth(img,yAxis.x  , yAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                                axisSampledY = self.get_depth(img,yAxis.x  , yAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            else:
                                brk =0
                                break
                            axisMeanX =[0,0,0]
                            axisMeanX[0] += axisSampledX[0]
                            axisMeanX[1] += axisSampledX[1]
                            axisMeanX[2] += axisSampledX[2]
                            totalSampledX+=1

                            axisMeanY =[0,0,0]
                            axisMeanY[0] += axisSampledY[0]
                            axisMeanY[1] += axisSampledY[1]
                            axisMeanY[2] += axisSampledY[2]
                            totalSampledY+=1
                        if brk ==0 :
                            break
                    if brk ==0:
                        continue
                    for i in range(len(axisMeanX)):
                        axisMeanX[i] = axisMeanX[i]/totalSampledX
                        axisMeanY[i] = axisMeanY[i]/totalSampledY
                    
                    
                    n_xAxis = Vector3(axisMeanX[0] - center3D[0], axisMeanX[1] - center3D[1], axisMeanX[2] - center3D[2])
                    n_yAxis = Vector3(axisMeanY[0] - center3D[0], axisMeanY[1] - center3D[1], axisMeanY[2] - center3D[2])
                    n_zAxis = np.cross([axisMeanX[0] - center3D[0], axisMeanX[1] - center3D[1], 
                    axisMeanX[2] - center3D[2]],[axisMeanY[0] - center3D[0], axisMeanY[1] - center3D[1], axisMeanY[2] - center3D[2]])
                    n_zAxis =Vector3(n_zAxis[0],n_zAxis[1],n_zAxis[2])
                    M =[ [n_xAxis.x, n_yAxis.x,n_zAxis.x,0],
                        [n_xAxis.y, n_yAxis.y, n_zAxis.y,0],
                        [n_xAxis.z, n_yAxis.z, n_zAxis.z,0],
                        [  0, 0, 0, 1 ]]
                    for a in range(3):
                        for b in range(3):
                            M[a][b] = M[a][b]/(max(M[0][a],M[1][a],M[2][a])- min(M[0][a],M[1][a],M[2][a]))
                    M = np.asarray(M)

                    q= tf.transformations.quaternion_from_matrix(M)
                    q*=tf.transformations.quaternion_from_euler(0, -3.14/2,-3.14/2)
                    q = q/(sum(q**2)**0.5 )
                    q = [0,0,1,0]
                    
                    pose_0 = PoseStamped()
                    pose_0.header.frame_id= "camera_depth_frame"
                    pose_0.pose.position.x = center3D[0]
                    pose_0.pose.position.y = center3D[1]
                    pose_0.pose.position.z = center3D[2]

                    pose_0.pose.orientation.x = q[0]
                    pose_0.pose.orientation.y = q[1]
                    pose_0.pose.orientation.z = q[2]
                    pose_0.pose.orientation.w = q[3]
                    
                    tes = tf.TransformListener()
                    while True:
                        try :
                            p_wrt_odom = tes.transformPose("/map", pose_0)
                            break
                        except:
                            continue
                    if "object_"+str(id+self.count) not in self.labels:  
                        self.L.append([p_wrt_odom,id +self.count])
                        self.labels.append("object_"+str(id+self.count))
            flag =1
            # print("Detection:",self.detect_time)

            # if self.detect_time == 20:
            #     self.detect_time = 0
            #     pub = rospy.Publisher("/check", String,  queue_size=1, latch=True)

            #     flag_off = String()
                
            #     flag_off.data = "stop"
                
            #     pub.publish(flag_off)
            #     flag=0
                #detect_time = 0 

            # self.detect_time +=flag        
                
        else:
            # print("Else of Depth Image Callback",self.count)
            self.count = len(self.labels)
            pub = rospy.Publisher('id_array', Int64MultiArray, queue_size=10,latch=True)
            ar = Int64MultiArray()
            # print(self.labels)
            # print(self.final_labels)
            for i in self.L:
                if self.same_object(i)[0]:
                    self.final_labels.append(i)
                else:
                    self.final_labels[self.same_object(i)[1]] = self.same_object(i)[2]
            # print([str(round(i[0].pose.position.x,2))+"_"+str(round(i[0].pose.position.y,2)) for i in self.final_labels])
            for i in self.final_labels:
                br = tf.TransformBroadcaster()
                br.sendTransform([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],
                                [i[0].pose.orientation.x,i[0].pose.orientation.y,i[0].pose.orientation.z,i[0].pose.orientation.w],
                                rospy.Time.now(),
                                "object_"+str(i[1]),
                                "map")
                ar.data.append(i[1])
            pub.publish(ar)

    def get_object_pose(self):
        self.boundingboxes = self.boundingboxes1
        pose_list = []
        pose_list2 = []
        try:
            img = self.bridge.imgmsg_to_cv2(self.image_data, "32FC1")
        except CvBridgeError as e:
            print(e)
        K = self.K
        points = [0,0]  
        if(len(self.boundingboxes.detections)):
            for i in range(len(self.boundingboxes.detections)):
                id = i 
                totalSampledX,totalSampledY,totalSampledCenter=0,0,0
                box_size_x,box_size_y = self.boundingboxes.detections[i].bbox.size_x,self.boundingboxes.detections[i].bbox.size_y

                center = Point()
                xAxis = Point()
                yAxis = Point()
                center.x = self.boundingboxes.detections[i].bbox.center.x
                center.y = self.boundingboxes.detections[i].bbox.center.y 
                center3D = self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                if self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                    center3D = self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                else:
                    continue

                if img.shape[1] > img.shape[0]:
                    side = "x"
                else:
                    side = "y"

                if self.boundingboxes.detections[i].bbox.size_y > self.boundingboxes.detections[i].bbox.size_x:
                    gripper_angle = math.pi
                else:
                    gripper_angle = math.pi/2
                
                num = 2 
                while True:
                    if side == "x":
                        if self.get_depth(img,center.x + img.shape[1]/num   , center.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                            points[0] = self.get_depth(img,center.x + img.shape[1]/num   , center.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            break
                    elif side=="y":
                        if self.get_depth(img,center.x + img.shape[1]/num   , center.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                            points[0] = self.get_depth(img,center.x , center.y + img.shape[0]/num , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            break
                    num+=1
                
                num = 2
                while True:
                    if side == "x":
                        if self.get_depth(img,center.x - img.shape[1]/num , center.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                            points[1] = self.get_depth(img,center.x - img.shape[1]/num , center.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            break
                    elif side=="y":
                        if self.get_depth(img,center.x  , center.y - img.shape[0]/num , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K):
                            points[1] = self.get_depth(img,center.x  , center.y - img.shape[0]/num , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                            break
                    num+=1

                    

                for i in points:
                    pose_0 = PoseStamped()
                    pose_0.header.frame_id= "camera_depth_frame"
                    pose_0.pose.position.x = i[0]
                    pose_0.pose.position.y = i[1]
                    pose_0.pose.position.z = i[2]

                    tes = tf.TransformListener()
                    while True:
                        try :
                            p_angle = tes.transformPose("/base_footprint", pose_0)
                            break
                        except:
                            continue
                    pose_list2.append(p_angle)

                x1,y1 = pose_list2[0].pose.position.x , pose_list2[0].pose.position.y
                x2,y2 = pose_list2[1].pose.position.x , pose_list2[1].pose.position.y

                angle =  math.atan2(y2 - y1, x2 - x1)

                pose_0 = PoseStamped()
                pose_0.header.frame_id= "camera_depth_frame"
                pose_0.pose.position.x = center3D[0]
                pose_0.pose.position.y = center3D[1]
                pose_0.pose.position.z = center3D[2]
                
                tes = tf.TransformListener()
                while True:
                    try :
                        p_wrt_odom = tes.transformPose("/base_footprint", pose_0)
                        break
                    except:
                        continue
                flag = True 
                for i in pose_list:
                    if abs(p_wrt_odom.pose.position.x) < .3 or abs(p_wrt_odom.pose.position.y) > .3:
                        flag = False
                        break
                    if abs(p_wrt_odom.pose.position.x-i[0].pose.position.x)<=0.05 and abs(p_wrt_odom.pose.position.y-i[0].pose.position.y)<=0.05:
                        flag= False
                        break
                if flag:
                    pose_list.append([p_wrt_odom, gripper_angle])

        return pose_list         

    def get_dustbin_pose(self):
        pose_list = []
        try:
            img = self.bridge.imgmsg_to_cv2(self.image_data, "32FC1")
        except CvBridgeError as e:
            print(e)
        K = self.K
        if(len(self.boundingboxes_d.detections)):
            for i in range(len(self.boundingboxes_d.detections)):
                id = i 
                totalSampledX,totalSampledY,totalSampledCenter=0,0,0
                box_size_x,box_size_y = self.boundingboxes_d.detections[i].bbox.size_x,self.boundingboxes_d.detections[i].bbox.size_y

                center = Point()
                xAxis = Point()
                yAxis = Point()
                center.x = self.boundingboxes_d.detections[i].bbox.center.x
                center.y = self.boundingboxes_d.detections[i].bbox.center.y 
                center3D = self.get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                                                
                pose_0 = PoseStamped()
                pose_0.header.frame_id= "camera_depth_frame"
                pose_0.pose.position.x = center3D[0]
                pose_0.pose.position.y = center3D[1]
                pose_0.pose.position.z = center3D[2]

                pose_0.pose.orientation.x = 0
                pose_0.pose.orientation.y = 0
                pose_0.pose.orientation.z = 0
                pose_0.pose.orientation.w = 1
                
                tes = tf.TransformListener()
                while True:
                    try :
                        p_wrt_odom = tes.transformPose("/base_footprint", pose_0)
                        break
                    except:
                        continue

                angle_diff = math.atan2(p_wrt_odom.pose.position.y, p_wrt_odom.pose.position.x)

                quats=quaternion_from_euler(0,0,angle_diff)

                db_base = PoseStamped()

                db_base.header.frame_id = "base_footprint"
                db_base.pose.position.x = p_wrt_odom.pose.position.x - (.4)*abs(math.cos(angle_diff))
                if p_wrt_odom.pose.position.y > 0:
                    db_base.pose.position.y = p_wrt_odom.pose.position.y - (.4)*abs(math.sin(angle_diff))
                else:
                    db_base.pose.position.y = p_wrt_odom.pose.position.y + (.4)*abs(math.sin(angle_diff))

                db_base.pose.position.z = .3
                db_base.pose.orientation.x = quats[0]
                db_base.pose.orientation.y = quats[1]
                db_base.pose.orientation.z = quats[2]
                db_base.pose.orientation.w = quats[3]

                while True:
                    try :
                        db_map = tes.transformPose("/map", db_base)
                        break
                    except:
                        continue

                final_quat = [db_map.pose.orientation.x, db_map.pose.orientation.y, db_map.pose.orientation.z, db_map.pose.orientation.w]

                return [db_map.pose.position.x, db_map.pose.position.y, final_quat]   
        else:
            return False                                 

    def listener(self, should_sub, item="trash"):
        if should_sub:
            self.obj_sub = rospy.Subscriber("/objects", Detection2DArray, self.objects_callback)
            self.check_sub = rospy.Subscriber("/check", String, self.callback)
            self.depth_img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.depth_image_callback, queue_size=10, tcp_nodelay=True)
            self.depth_info_sub = rospy.Subscriber("/camera/depth/camera_info",CameraInfo, self.image_info_callback)
        else:
            self.obj_sub.unregister()
            self.check_sub.unregister()
            self.depth_img_sub.unregister()
            self.depth_info_sub.unregister()

    def listener3(self):
        rospy.init_node('test2')
        self.obj_sub = rospy.Subscriber("/objects", Detection2DArray, self.objects_callback)
        self.check_sub = rospy.Subscriber("/check", String, self.callback)
        self.depth_img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.depth_image_callback)
        self.depth_info_sub = rospy.Subscriber("/camera/depth/camera_info",CameraInfo, self.image_info_callback)
        rospy.spin()

    def listener_2(self):
        # rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.depth_image_callback2 )
        rospy.Subscriber("/objects", Detection2DArray, self.objects_callback)
        rospy.Subscriber("/camera/depth/camera_info",CameraInfo, self.image_info_callback)
        # rospy.spin()

    def listener_dustbin(self, should_sub=True):
        if should_sub:
            self.img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.depth_image_callback2 )
            self.object_sub_d = rospy.Subscriber("/objects_d", Detection2DArray, self.objects_callback_d, tcp_nodelay=True)
            self.info_sub = rospy.Subscriber("/camera/depth/camera_info",CameraInfo, self.image_info_callback)
        else:
            self.img_sub.unregister()
            self.object_sub.unregister()
            self.info_sub.unregister()

if __name__ == '__main__':
    test_object = fusion()
    test_object.listener3()
    
    