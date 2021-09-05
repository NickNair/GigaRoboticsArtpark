#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf
from std_msgs.msg import String,Int64MultiArray
from geometry_msgs.msg import *
from vision_msgs.msg import Detection2DArray

def same_object(labels,Object):
    count = 0
    for i in labels:
        if abs(Object[0].pose.position.x-i[0].pose.position.x)<=0.05 and abs(Object[0].pose.position.y-i[0].pose.position.y)<=0.05:
            # print("Object ", Object[2]," is very close to Object ", id)
            # final_labels[count] = Object
            return False
        count+=1
    return True 

def objects_callback(data):
    global boundingboxes
    boundingboxes = data

def image_info_callback(data):
    global K 
    K = 1/data.K[4]


def get_depth(img,x,y,cx,cy,fx,fy):
    center_x , center_y = cx,cy
    #print(type(img))
    unit_scaling = 1
    constant_x = unit_scaling/fx
    constant_y = unit_scaling/fy
    depth = img[int(y)][int(x)]
    #print(depth)
    return [ (x - center_x)*depth*constant_x ,(y - center_y)*depth*constant_y , depth*unit_scaling]



def depth_image_callback(data):
    global K,boundingboxes,labels,check,L,count,final_labels,detect_time
    if(check=="Detect"):
        print("If of camera callback")
        Lock = True
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
        if(len(boundingboxes.detections)):
            for i in range(len(boundingboxes.detections)):
                id = i 
                totalSampledX,totalSampledY,totalSampledCenter=0,0,0
                box_size_x,box_size_y = boundingboxes.detections[i].bbox.size_x,boundingboxes.detections[i].bbox.size_y

                center = Point()
                xAxis = Point()
                yAxis = Point()
                center.x = boundingboxes.detections[i].bbox.center.x
                center.y = boundingboxes.detections[i].bbox.center.y 
                center3D = get_depth(img,center.x +0.5 , center.y + 0.5, img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
            
                for j in range(int(box_size_x)):
                    for k in range(4):
                        xAxis.x, xAxis.y = center.x + j ,center.y + k
                        axisSampledX = get_depth(img,xAxis.x  , xAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                        yAxis.x, yAxis.y = center.x+ k, center.y - j
                        axisSampledY = get_depth(img,yAxis.x  , yAxis.y  , img.shape[1]/2 - 0.5, img.shape[0]/2 -0.5, 1/K,1/K)
                        
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
                print(p_wrt_odom)
                # br = tf.TransformBroadcaster()
                # br.sendTransform([p_wrt_odom.pose.position.x,p_wrt_odom.pose.position.y,p_wrt_odom.pose.position.z],
                            # [p_wrt_odom.pose.orientation.x,p_wrt_odom.pose.orientation.y,p_wrt_odom.pose.orientation.z,p_wrt_odom.pose.orientation.w],
                            # rospy.Time.now(),
                            # "object_"+str(id+count),
                            # "odom")
                if "object_"+str(id+count) not in labels:  
                    L.append([p_wrt_odom,id +count])
                    labels.append("object_"+str(id+count))
        flag =1
        print("Detection:",detect_time)
        if detect_time == 5:
            pub = rospy.Publisher("/check", String,  queue_size=1)

            flag_off = String()
            
            flag_off.data = "stop"
            
            pub.publish(flag_off)
            flag=0
            #detect_time = 0 

        detect_time +=flag

        
                
                
    else:
        print("Else of Depth Image Callback",count)
        count = len(labels)
        pub = rospy.Publisher('id_array', Int64MultiArray, queue_size=1)
        ar = Int64MultiArray()
        print(labels)
        for i in L:
            if same_object(final_labels,i):
                final_labels.append(i)
        print([str(round(i[0].pose.position.x,2))+"_"+str(round(i[0].pose.position.y,2)) for i in final_labels])
        for i in final_labels:
            br = tf.TransformBroadcaster()
            br.sendTransform([i[0].pose.position.x,i[0].pose.position.y,i[0].pose.position.z],
                            [i[0].pose.orientation.x,i[0].pose.orientation.y,i[0].pose.orientation.z,i[0].pose.orientation.w],
                            rospy.Time.now(),
                            "object_"+str(i[1]),
                            "map")
            ar.data.append(i[1])
        pub.publish(ar)


            


def callback(data):
    global check,L
    check = data.data

def id_callback(data):
    print("In ID Callback")
    global trans,L2,Lock
    if Lock == True:
        ids = data.data
        tf.Transp
        listener = tf.TransformListener()
        for i in ids:
            while True:
                try:
                    t,r =listener.lookupTransform("/map","object_"+str(i), rospy.Time(0))
                    break
                except:
                    continue
            L2.append([t,r,i])
    Lock = False
    
    pub = rospy.Publisher('id_array', Int64MultiArray, queue_size=1)
    ar = Int64MultiArray()
    for i in L2:
        br = tf.TransformBroadcaster()
        br.sendTransform(i[0],
        i[1],
        rospy.Time.now(),
        "object_"+str(i[2]),
        "odom")
        ar.data.append(i[2])
        rospy.sleep(0.002)
    pub.publish(ar)
        


    
def image_callback5(data):
    pass
    #print(data)
    # print("Henlo")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/objects", Detection2DArray, objects_callback)
    rospy.Subscriber("/check", String, callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image,depth_image_callback )
    rospy.Subscriber("/camera/depth/camera_info",CameraInfo, image_info_callback)
    rospy.spin()

if __name__ == '__main__':
    global labels,L,count,final_labels, detect_time
    detect_time = 0 
    final_labels =[]
    L,count,labels = [], 0 ,[]
    listener()

    
