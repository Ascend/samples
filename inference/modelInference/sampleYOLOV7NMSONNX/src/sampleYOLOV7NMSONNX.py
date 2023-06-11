"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2023-05-25 09:12:13
MODIFIED: 2023-05-25 10:10:55
"""
import os
import numpy as np
from label import labels
import cv2
from acllite_imageproc import AclLiteImage
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
from acllite_logger import log_info


class sample_YOLOV7_NMS_ONNX(object):
    def __init__(self, yolo_model_path, yolo_model_width, yolo_model_height, postprocess_model_path):
        self.yolo_model_path = yolo_model_path    # string
        self.yolo_model = None
        self.yolo_model_width = yolo_model_width
        self.yolo_model_height = yolo_model_height
        self.yolo_result = None
        self.postprocess_model = None
        self.postprocess_model_path = postprocess_model_path
        self.postprocess_input = None
        self.postprocess_result = None
        self.resource = None
        self.dvpp = None
        self.image = None
        self.resized_image = None

    def init_resource(self):
        # init acl resource
        self.resource = AclLiteResource()
        self.resource.init()

        # init dvpp resource
        self.dvpp = AclLiteImageProc(self.resource)

        # load yolo model from file
        self.yolo_model = AclLiteModel(self.yolo_model_path)
        
        # load postprocess model from file
        self.postprocess_model = AclLiteModel(self.postprocess_model_path)

    def yolo_process_input(self, input_path):
        # read image from file
        self.image = AclLiteImage(input_path)

        # memory copy from host to dvpp
        image_input = self.image.copy_to_dvpp()

        # decode image from JPEGD format to YUV
        yuv_image = self.dvpp.jpegd(image_input)

        # execute resize
        self.resized_image = self.dvpp.crop_and_paste(yuv_image, self.image.width, self.image.height, 
                                                      self.yolo_model_width, self.yolo_model_height)

    def postprocess_process_input(self):
        # construct image info
        image_info = np.array([self.yolo_model_width, self.yolo_model_height,
                               self.yolo_model_width, self.yolo_model_height],
                               dtype=np.float32)
        
        self.yolo_result.reverse()
        
        # construct postprocess input
        self.postprocess_input = [*self.yolo_result, image_info]  
        
    def yolo_inference(self):
        # inference
        self.yolo_result = self.yolo_model.execute([self.resized_image])  
        
    def postprocess_inference(self):
        self.postprocess_result = self.postprocess_model.execute(self.postprocess_input)
    
    def postprocess_get_reslut(self, src_image_path):
        box_num = self.postprocess_result[1][0, 0]
        box_info = self.postprocess_result[0].flatten()
        src_image = cv2.imread(src_image_path)
        scale_x = src_image.shape[1] / self.yolo_model_width
        scale_y = src_image.shape[0] / self.yolo_model_height
        
        # get scale factor
        if scale_x > scale_y:
            max_scale = scale_x
        else:
            max_scale = scale_y
        colors = [0, 0, 255]
        
        # draw the boxes in original image 
        for n in range(int(box_num)):
            ids = int(box_info[5 * int(box_num) + n])
            score = box_info[4 * int(box_num) + n]
            label = labels[ids] + ":" + str("%.2f" % score)
            top_left_x = box_info[0 * int(box_num) + n] * max_scale
            top_left_y = box_info[1 * int(box_num) + n] * max_scale
            bottom_right_x = box_info[2 * int(box_num) + n] * max_scale
            bottom_right_y = box_info[3 * int(box_num) + n] * max_scale
            cv2.rectangle(src_image, (int(top_left_x), int(top_left_y)),
                         (int(bottom_right_x), int(bottom_right_y)), colors)
            p3 = (max(int(top_left_x), 15), max(int(top_left_y), 15))
            cv2.putText(src_image, label, p3, cv2.FONT_ITALIC, 0.6, colors, 1)
        output_path = os.path.join("../out", os.path.basename(src_image_path))
        cv2.imwrite(output_path, src_image)
    
    def release_resource(self):
        # release resource includes acl resource, data set and unload model
        self.dvpp.__del__()
        self.yolo_model.__del__()
        self.postprocess_model.__del__()
        self.resource.__del__()
        AclLiteResource.__del__ = lambda x: 0
        AclLiteImage.__del__ = lambda x: 0
        AclLiteImageProc.__del__ = lambda x: 0
        AclLiteModel.__del__ = lambda x: 0


if __name__ == "__main__":
    yolo_width = 640
    yolo_height = 640
    current_dir = os.path.dirname(os.path.abspath(__file__))
    yolo_model_path = os.path.join(current_dir, "../model/yolov7x.om")
    postprocess_model_path = os.path.join(current_dir, "../model/postprocess.om")
    images_path = os.path.join(current_dir, "../data")
    if not os.path.exists(yolo_model_path):
        raise Exception("the yolo model path is not exist")
    
    if not os.path.exists(postprocess_model_path):
        raise Exception("the postprocess model path is not exist")
    
    if not os.path.exists(images_path):
        raise Exception("the images path is not exist")
    
    all_path = []
    for path in os.listdir(images_path):
        if path != '.keep':
            total_path = os.path.join(images_path, path)
            all_path.append(total_path)
    
    if len(all_path) == 0:
        raise Exception("the directory is empty, please download image")
    net = sample_YOLOV7_NMS_ONNX(yolo_model_path, yolo_width, yolo_height, postprocess_model_path)
    net.init_resource()
    for images_path in all_path:
        net.yolo_process_input(images_path)
        net.yolo_inference()
        net.postprocess_process_input()
        net.postprocess_inference()
        net.postprocess_get_reslut(images_path)
    net.release_resource()
    log_info("success")