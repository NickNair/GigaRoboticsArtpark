import numpy as np
import argparse
import cv2 as cv
import subprocess
import time
import os

def same_object1(boxes,x,y,w,h):
    for x_,y_,w_,h_,id in boxes:
        if abs(x-x_)<=10 and abs(y-y_)<=10 and abs(h-h_)<=10 and abs(w-w_)<=10:
            return id
    return False


def draw_boxes(img, boxes, confidences, classids, idxs, colors, labels):
    if len(idxs) > 0:
        for i in idxs.flatten():
            x, y = boxes[i][0], boxes[i][1]
            w, h = boxes[i][2], boxes[i][3]
    
            #color = [int(c) for c in colors[classids[i]]]


            cv.rectangle(img, (x, y), (x+w, y+h), (0,0,0), 2)
            if(same_object1(labels,x,y,w,h)):
                text = str(same_object1(labels,x,y,w,h))
                cv.putText(img, text, (x, y-5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

    return img
def same_object(boxes,x,y,w,h):
    if len(boxes)==0:
        return True
    for x_,y_,w_,h_ in boxes:
        if abs(x-x_)<=10 and abs(y-y_)<=10 and abs(h-h_)<=10 and abs(w-w_)<=10:
            return False
            
    return True
 
def generate_boxes(outs, height, width, tconf,labels):
    boxes = []
    confidences = []
    classids = []
    center=[]

    for out in outs:
        for detection in out:
            scores = detection[5:]
            classid = np.argmax(scores)
            confidence = scores[classid]
            
            if confidence > tconf:

                box = detection[0:4] * np.array([width, height, width, height])
                centerX, centerY, bwidth, bheight = box.astype('int')


                x = int(centerX - (bwidth / 2))
                y = int(centerY - (bheight / 2))

                if(same_object(boxes,x,y,int(bwidth), int(bheight))):
                    boxes.append([x, y, int(bwidth), int(bheight)])
                    center.append(np.array([[x], [y]]))
                    confidences.append(float(confidence))
                    classids.append(classid)

    return boxes, confidences, classids,center

def infer_image(net, layer_names, height, width, img, colors, labels, boxes=None, confidences=None, classids=None, idxs=None, infer=True):
    print("hi")
    if infer:

        blob = cv.dnn.blobFromImage(img, 1 / 255.0, (416, 416), swapRB=True, crop=False)


        net.setInput(blob)
        outs = net.forward(layer_names)
       
        boxes, confidences, classids,center = generate_boxes(outs, height, width, 0.5,labels)
        

        idxs = cv.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

    if boxes is None or confidences is None or idxs is None or classids is None:
        raise 'error'
        
    # Draw labels and boxes on the image
    img = draw_boxes(img, boxes, confidences, classids, idxs, colors, labels)

    return img, boxes,center, confidences, classids, idxs

