import numpy as np
import cv2 as cv
from yolo2_utils import infer_image


if __name__ == '__main__':
	labels = ["cans"]
	colors = np.random.randint(0, 255, size=(len(labels), 3), dtype='uint8')
	net = cv.dnn.readNetFromDarknet('/artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/scripts/door.cfg', '/artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/scripts/door.weights')
	layer_names = net.getLayerNames()
	layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
	image = cv.imread("/artpark_workspace/src/GigaRoboticsArtpark/apbot_perception/scripts/door1.jpeg")
	height, width = image.shape[:2]
	frame, box, _, _, _ , _= infer_image(net, layer_names, height, width, image, colors, labels)
	print(box)
	cv.imshow("frame",frame)
	cv.waitKey(0)

	
	
	
