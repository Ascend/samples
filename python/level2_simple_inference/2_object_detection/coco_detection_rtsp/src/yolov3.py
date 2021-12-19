"""yolov3"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from io import BytesIO
import os
import sys

from acllite_model import AclLiteModel

LABEL = 1
SCORE = 2
TOP_LEFT_X = 3
TOP_LEFT_Y = 4
BOTTOM_RIGHT_X = 5
BOTTOM_RIGHT_Y = 6
labels = ["person",
        "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
        "pizza", "donut", "cake", "chair", "sofa", "potted plant", "bed", "dining table",
        "toilet", "TV monitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]

class Yolov3(object):
    """yolov3"""
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self._acl_resource = acl_resource
        self._model_width = model_width
        self._model_height = model_height
        self._model = AclLiteModel(model_path)

    def __del__(self):
        if self._model:
            del self._model

    def construct_image_info(self):
        """construct"""
        image_info = np.array([self._model_width, self._model_height,
                       self._model_width, self._model_height],
                       dtype = np.float32)
        return image_info

    def execute(self, data):
        """execute"""
        image_info = self.construct_image_info()
        return self._model.execute([data.resized_image, image_info])       
 
    def post_process(self, infer_output, data):
        """post"""
        print("infer output shape is : ", infer_output[1].shape)
        box_num = int(infer_output[1][0, 0])
        print("box num = ", box_num)
        box_num = infer_output[1][0, 0]
        box_info = infer_output[0].flatten()
        scalex = data.frame_width / self._model_width
        scaley = data.frame_height / self._model_height
        if scalex > scaley:
            scaley =  scalex
        detection_result_list = []
        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            label = labels[ids]
            score = box_info[4 * int(box_num)+n]
            lt_x = int(box_info[0 * int(box_num)+n] * scaley)
            lt_y = int(box_info[1 * int(box_num)+n] * scaley)
            rb_x = int(box_info[2 * int(box_num) + n] * scaley)
            rb_y = int(box_info[3 * int(box_num) + n] * scaley)
            print("channel %d inference result: box top left(%d, %d), "
                  "bottom right(%d %d), score %s" % (data.channel, 
                  lt_x, lt_y, rb_x, 
                  rb_y, score))
        return detection_result_list
