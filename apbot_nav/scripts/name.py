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

class BoundingBoxes:
    def __init__(self, item="trash_marking", trash_marking=False, trash_marking_pick=False, item_specific="trash", isDustbin=False):
        self.trash_marking=trash_marking
        self.item_specific = item_specific 
        self.trash_marking_pick = trash_marking_pick
        self.item = item
        self.isDustbin = isDustbin
        self.bridge = CvBridge()
        self.check = ""
        self.image_pub = rospy.Publisher('output', Image,queue_size=1, latch=True)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1, latch=True)
        self.object_pub_dustbin = rospy.Publisher("objects_d", Detection2DArray, queue_size=1)
        self.labels_array = ['trash','trash','markings']
        self.labels = []
        self.classes= []
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')
        self.net = cv.dnn.readNetFromDarknet('artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/config/' + item + '.cfg', 'artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/weights/' + item + '.weights')
        self.layer_names = self.net.getLayerNames()
        self.layer_names = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        
        rospy.sleep(.1)
        
    def get_ID(self,box):
    	if same_object(self.labels,box[0],box[1],box[2],box[3]):
    	    return  same_object(self.labels,box[0],box[1],box[2],box[3])
    	self.labels.append([box[0],box[1],box[2],box[3],box[0]])
    	return box[0]  	 

    def image_callback(self, data):
        objArray = Detection2DArray()

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        
        height, width = img.shape[:2]
        img, box, _, confidences, classids, idxs = infer_image(self.net, self.layer_names, height, width, img, self.colors, self.labels)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        objArray.detections =[]
        objArray.header=data.header
        object_count=1

        for i in range(len(box)):
            if self.trash_marking:
                object_count+=1
                objArray.detections.append(object_predict(box[i],data.header,classids[i],confidences[i],img))

            elif self.trash_marking_pick:
                if( self.labels_array[classids[i]] == self.item_specific):
                    object_count+=1
                    objArray.detections.append(object_predict(box[i],data.header,classids[i],confidences[i],img))

            elif self.isDustbin:
                cropped_img = img[box[i][1]:box[i][1] + box[i][3], box[i][0]:box[i][0]+box[i][2] ]
                hsv = cv.cvtColor(cropped_img, cv.COLOR_BGR2HSV)
                mask = cv.inRange(hsv, (36, 25, 25), (86, 255,255))
                imask = mask>0
                green = np.zeros_like(cropped_img, np.uint8)
                green[imask] = cropped_img[imask]
                m = green.mean(axis=0).mean(axis=0)

                if m[0]>25 or m[1]>25 or m[2]>25:
                    object_count+=1
                    objArray.detections.append(object_predict(box[i],data.header,self.get_ID(box[i]),confidences[i],img))                
            
            else:
                object_count+=1
                objArray.detections.append(object_predict(box[i],data.header,self.get_ID(box[i]),confidences[i],img))

        if self.isDustbin:
            self.object_pub_dustbin.publish(objArray)
        else:
            self.object_pub.publish(objArray)

        # cv.imshow("Image window", green)
        # cv.imshow("full img", img)
        # cv.waitKey(3)

    def callback(self,data):
        self.check = data.data
    
    def listener2(self):
        rospy.init_node('test_node')
        rospy.Subscriber("/check", String, self.callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

    def listener(self, status=1):
        if status:
            self.check_sub = rospy.Subscriber("/check", String, self.callback)
            self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        else:
            self.check_sub.unregister()
            self.img_sub.unregister()

if __name__ == '__main__':
    obj = BoundingBoxes(item="trash_marking", trash_marking=True)
    # obj.check = "Detect"
    obj.listener2()
    # start_yolo()
    pass