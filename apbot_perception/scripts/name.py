#!/usr/bin/env python3

import rospy

import numpy as np
import cv2 as cv
from yolo2_utils import infer_image

from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

def object_predict(object_data, header, classid,confidence,image):
    image_height,image_width,channels = image.shape
    obj=Detection2D()
    obj_hypothesis= ObjectHypothesisWithPose()

    object_id= classid
    object_score=confidence
    #dimensions=object_data

    obj.header=header
    obj_hypothesis.id = object_id
    obj_hypothesis.score = object_score
    obj.results.append(obj_hypothesis)
    obj.bbox.size_y = int(object_data[3])
    obj.bbox.size_x = int(object_data[2])
    obj.bbox.center.x = int(object_data[0]) + int(object_data[2])//2 
    obj.bbox.center.y = int(object_data[1]) + int(object_data[3])//2

    return obj


def same_object(boxes,x,y,w,h):
    for x_,y_,w_,h_,id in boxes:
        if abs(x-x_)<=10 and abs(y-y_)<=10 and abs(h-h_)<=10 and abs(w-w_)<=10:
            return id
    return False



class boundingboxes:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('output', Image,queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.labels = []

        rospy.sleep(.1)
        
    def get_ID(self,box):
    	if same_object(self.labels,box[0],box[1],box[2],box[3]):
    	    return  same_object(self.labels,box[0],box[1],box[2],box[3])
    	self.labels.append([box[0],box[1],box[2],box[3],box[0]])
    	return box[0]  	 

    def image_callback(self, data):  
        global labels,colors,net,layer_names 

        objArray = Detection2DArray()

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        
        height, width = img.shape[:2]
        img, box, _, confidences, classids, idxs = infer_image(net, layer_names, height, width, img, colors, self.labels)
        print(box)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        objArray.detections =[]
        objArray.header=data.header
        object_count=1

        for i in range(len(box)):
            object_count+=1
            objArray.detections.append(object_predict(box[i],data.header,self.get_ID(box[i]),confidences[i],img))

        self.object_pub.publish(objArray)


        cv.imshow("Image window", img)
        cv.waitKey(3)

    
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

def start_yolo():
    global labels,colors,net,layer_names
    labels = []
    colors = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')
    net = cv.dnn.readNetFromDarknet('/artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/config/trash.cfg', '/artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/weights/trash.weights')
    layer_names = net.getLayerNames()
    layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    print("Loaded weights")
    obj = boundingboxes()
    obj.listener()
    rate = rospy.Rate(10)

if __name__ == '__main__':
    start_yolo()
