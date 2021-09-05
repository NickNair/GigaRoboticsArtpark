#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf
from std_msgs.msg import String,Int64MultiArray
from geometry_msgs.msg import *
from vision_msgs.msg import Detection2DArray


def objects_callback(data):
    global boundingboxes
    boundingboxes = data

def image_info_callback(data):
    global K 
    K = 1/data.K[4]

def image_callback3(data):
    pass
    #print(data)
    # print("Henlo")

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
    global K,boundingboxes,labels,labels2,check,L
    if(check=="Detect"):
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
                
                br = tf.TransformBroadcaster()
                br.sendTransform((center3D[0], center3D[1], center3D[2]),
                            q,
                            rospy.Time.now(),
                            "object_"+str(id),
                            "camera_depth_frame")
                if "object_"+str(id) not in labels:  
                    L.append([center3D[0], center3D[1], center3D[2],q,"object_"+str(id)])
                    labels.append("object_"+str(id))
                
    else:
        print(len(L))
        pub = rospy.Publisher('id_array', Int64MultiArray, queue_size=1)
        ar = Int64MultiArray()
        
        for i in L:
            br = tf.TransformBroadcaster()
            br.sendTransform((i[0], i[1], i[2]),
            i[3],
            rospy.Time.now(),
            i[4],
            "camera_depth_frame")
            ar.data.append(int(i[4][7:]))
            rospy.sleep(0.002)
        pub.publish(ar)

            


def callback(data):
    global check,L
    check = data.data


        


    
def image_callback5(data):
    pass
    #print(data)
    # print("Henlo")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/objects", Detection2DArray, objects_callback)
    rospy.Subscriber("/check", String, callback)
    #rospy.Subscriber("/camera/color/image_raw", Image, image_callback2)
    #rospy.Subscriber("/camera/color/camera_info", CameraInfo, image_callback3)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image,depth_image_callback )
    rospy.Subscriber("/camera/depth/camera_info",CameraInfo, image_info_callback)
    rospy.spin()

if __name__ == '__main__':
    global labels,labels2,L
    L=[]
    labels=[]
    labels2=[]
    listener()

    
