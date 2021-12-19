"""
Copyright 2020 Huawei Technologies Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import numpy as np
import sys
import os

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, "../../../common/"))
print(sys.path)

import cv2
import datetime
import argparse

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource 


def PostProcessing(image, resultList, threshold=0.6):
	"""draw the bounding boxes for all detected hands with confidence greater than a set threshold"""
	num_detections = resultList[0][0].astype(np.int)
	scores = resultList[2]
	boxes = resultList[3]
	bbox_num = 0
	
	# loop through all the detections and get the confidence and bbox coordinates
	for i in range(num_detections):
		det_conf = scores[0, i]
		det_ymin = boxes[0, i, 0]
		det_xmin = boxes[0, i, 1]
		det_ymax = boxes[0, i, 2]
		det_xmax = boxes[0, i, 3]

		bbox_width = det_xmax - det_xmin
		bbox_height = det_ymax - det_ymin
		# the detection confidence and bbox dimensions must be greater than a minimum value to be a valid detection
		if threshold <= det_conf and 1 >= det_conf and bbox_width > 0 and bbox_height > 0:
			bbox_num += 1
			xmin = int(round(det_xmin * image.shape[1]))
			ymin = int(round(det_ymin * image.shape[0]))
			xmax = int(round(det_xmax * image.shape[1]))
			ymax = int(round(det_ymax * image.shape[0]))
			
			cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
		else:
			continue

	print("detected bbox num:", bbox_num)
	SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
	Output_PATH = os.path.join(SRC_PATH, "../output/output.jpg")
	try:
		os.mkdir(os.path.join(SRC_PATH, "../output/"))
	except Exception as e:
		print("Output Path already exists")
	cv2.imwrite(Output_PATH, image)

	
def PreProcessing(image):
	"""Pre-processing - resize image to 300x300 RGB"""
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	image = cv2.resize(image, (300, 300))
	# type conversion to UINT8
	image = image.astype(np.uint8).copy()
	return image


if __name__ == '__main__':

	description = 'hand detection'
	parser = argparse.ArgumentParser(description=description)
	parser.add_argument('--input_image', type=str, default='../data/hand.jpg', help="Directory path for image")

	args = parser.parse_args()
	
	model_name = 'Hand_detection'
	img_file = args.input_image

	# initialize acl resource
	acl_resource = AclLiteResource()
	acl_resource.init()

	#load model
	PROJECT_SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
	MODEL_PATH = os.path.join(PROJECT_SRC_PATH, "../model/" + model_name + ".om")
	print("MODEL_PATH:", MODEL_PATH)
	try:
		model = AclLiteModel(MODEL_PATH)
		# model = AclLiteModel(acl_resource, MODEL_PATH)
	except Exception as e:
		print("AclLiteModel loads error from", MODEL_PATH)

	# load image file 
	img_path = os.path.join(path, args.input_image)
	test_image = cv2.imread(img_path)
	input_image = PreProcessing(test_image)
	
	# om model inference 
	inferenceList  = model.execute([input_image])

	# postprocessing and save inference results
	PostProcessing(test_image, inferenceList)
	



