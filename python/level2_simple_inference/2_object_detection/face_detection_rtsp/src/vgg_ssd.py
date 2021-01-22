import numpy as np
from PIL import Image, ImageDraw, ImageFont
from io import BytesIO
import os
import sys

from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.presenteragent import presenter_datatype
from atlas_utils.acl_model import Model

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
        #加载离线模型
        self._model = Model(model_path)

    def __del__(self):
        if self._model:
            del self._model

    def execute(self, data):
        #将数据送入离线模型推理
        return self._model.execute([data.resized_image,])       
 
    def post_process(self, infer_output, data):
        #vgg ssd有两个输出,第一个输出infer_output[0]为检测到的物体个数,shape为(1,8)
        box_num = int(infer_output[0][0, 0])

       #第二个输出infer_output[1]为检测到的物体信息,shape为(1, 200, 8)
        box_info = infer_output[1][0]  
        detection_result_list = []

        for i in range(box_num):
            #检测到的物体置信度
            score = box_info[i, SCORE]
            if score < 0.9:
                break 

            detection_item = presenter_datatype.ObjectDetectionResult()            
            detection_item.confidence = score
            #人脸位置框坐标, 是归一化的坐标，需要乘以图片宽高转换为图片上的坐标
            detection_item.box.lt.x = int(box_info[i, TOP_LEFT_X] * data.frame_width)
            detection_item.box.lt.y = int(box_info[i, TOP_LEFT_Y] * data.frame_height)
            detection_item.box.rb.x = int(box_info[i, BOTTOM_RIGHT_X] * data.frame_width)
            detection_item.box.rb.y = int(box_info[i, BOTTOM_RIGHT_Y] * data.frame_height)
            #将置信度组织为字符串
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
                   



   