"""
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
import cv2
import os
import copy
import argparse
import sys
sys.path.append("../../../common/")

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource 

import yolov3.yolov3 as YOLOV3
import whenet.whenet as WHENET


def softmax(x):
    """Softmax"""
    x -= np.max(x, axis=1, keepdims=True)
    a = np.exp(x)
    b = np.sum(np.exp(x), axis=1, keepdims=True)
    return a / b


def PreProcessing_pose(image):
    """head pose estimation preprocessing"""
    # convert image to float type
    image = image.astype('float32')
    image = image / 255.
    # resize image to 224X224
    image = cv2.resize(image, (224, 224))
    return image


def main(): 
    """main"""
    description = 'head pose estimation'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--input_image', type=str, default='../data/test.jpg', help="Directory path for image")
    args = parser.parse_args()
    DATA_PATH = args.input_image

    YOLO_Model_Name = 'yolo_model'
    WHENet_Model_Name = 'WHENet_b2_a1_modified'

    SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
    YOLO_PATH = os.path.join(SRC_PATH, "../model/" + YOLO_Model_Name + ".om")
    print("YOLO MODEL_PATH:", YOLO_PATH)
    WHENet_MODEL_PATH = os.path.join(SRC_PATH, "../model/" + WHENet_Model_Name + ".om")
    print("WHENet MODEL_PATH:", WHENet_MODEL_PATH)

    #initialize acl runtime 
    acl_resource = AclLiteResource()
    acl_resource.init()

    #read image
    image = cv2.imread(DATA_PATH)
    image_height, image_width = image.shape[0], image.shape[1]
    
    # loading and initializing models
    yolo_model = AclLiteModel(YOLO_PATH)

    yolo_v3 = YOLOV3.YOLOV3(image_height, image_width, yolo_model)
    whenet_model = AclLiteModel(WHENet_MODEL_PATH)
    whenet = WHENET.WHENet(image_height, image_width, whenet_model)
    
    # convert image to RGB 
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # yolov3: inferencing
    nparryList, boxList  = yolo_v3.inference(image)

    # plot yolo and whenet detection result on the image
    save_img = True
    if save_img:
        image_res = copy.deepcopy(image)
    # whenet: preprocessing, transpose, inference, postprocessing
    detection_result_list = []

    for i in range(len(nparryList)):
        box_width, box_height = (boxList[i][0]+boxList[i][1]) / 2, (boxList[i][2] + boxList[i][3]) / 2
        detection_item = whenet.inference(PreProcessing_pose(nparryList[i]), box_width, box_height)
        if save_img:
            #plot head detection box from yolo predictions
            cv2.rectangle(image_res, (boxList[i][0], boxList[i][2]), 
                (boxList[i][1], boxList[i][3]), (127, 125, 125), 2)
            #plot head pose detection lines from whenet predictions
            cv2.line(image_res, (int(box_width), int(box_height)), 
                (int(detection_item["yaw_x"]), int(detection_item["yaw_y"])), (255, 0, 0), 4)
            cv2.line(image_res, (int(box_width), int(box_height)), 
                (int(detection_item["pitch_x"]), int(detection_item["pitch_y"])), (0, 255, 0), 4)
            cv2.line(image_res, (int(box_width), int(box_height)), 
                (int(detection_item["roll_x"]), int(detection_item["roll_y"])), (0, 0, 255), 4)

    if save_img:
        image_res = cv2.cvtColor(image_res, cv2.COLOR_RGB2BGR)
        SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
        Output_PATH = os.path.join(SRC_PATH, "../output/test_output.jpg")
        try:
            os.mkdir(os.path.join(SRC_PATH, "../output/"))
        except OSError:
            print("Output Path already exists")
        cv2.imwrite(Output_PATH, image_res)


if __name__ == "__main__":
    main()    
    




