import numpy as np
from PIL import Image, ImageDraw, ImageFont
from io import BytesIO
import os
import sys

from presenteragent import presenter_datatype
from acllite_model import AclLiteModel

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6

class VggSsd(object):
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        self._model = AclLiteModel(model_path)

    def __del__(self):
        if self._model:
            del self._model

    def execute(self, data):
        #Send data to the model for inference
        return self._model.execute([data.resized_image,])       
 
    def post_process(self, infer_output, data):
        #vgg ssd has two outputs, the first output 
        # infer_output[0] is the number of detected objects, and the shape is (1,8)
        box_num = int(infer_output[0][0, 0])
        #The second output infer_output[1] is the detected object information, the shape is (1, 200, 8)
        box_info = infer_output[1][0]  
        detection_result_list = []

        for i in range(box_num):
            #Detected object confidence
            score = box_info[i, SCORE]
            if score < 0.9:
                break 

            detection_item = presenter_datatype.ObjectDetectionResult()            
            detection_item.confidence = score
            #Person face position frame coordinates, normalized coordinates, 
            # need to be multiplied by the width and height of the picture to convert to the coordinates on the picture
            detection_item.box.lt.x = int(box_info[i, TOP_LEFT_X] * data.frame_width)
            detection_item.box.lt.y = int(box_info[i, TOP_LEFT_Y] * data.frame_height)
            detection_item.box.rb.x = int(box_info[i, BOTTOM_RIGHT_X] * data.frame_width)
            detection_item.box.rb.y = int(box_info[i, BOTTOM_RIGHT_Y] * data.frame_height)
            #Organize the confidence into a string
            detection_item.result_text = str(round(detection_item.confidence * 100, 2)) + "%"
            detection_result_list.append(detection_item)

        self.print_detection_results(detection_result_list, data.channel)   

        return detection_result_list

    def print_detection_results(self, results, channel_id):
        for item in results:
            print("channel %d inference result: box top left(%d, %d), "
                  "bottom right(%d %d), score %s"%(channel_id, 
                  item.box.lt.x, item.box.lt.y, item.box.rb.x, 
                  item.box.rb.y, item.result_text))
