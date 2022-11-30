
"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import cv2

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))

import acl
import acllite_utils as utils
import constants as constants
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from PIL import Image, ImageDraw, ImageFont
import numpy as np

class CrowdCount(object):
    """
    crowdCount
    """
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._model = None
        self.run_mode  = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = None
        self._model = None

    def init(self):    
        """
        init
        """ 
        self._dvpp = AclLiteImageProc() 
        self._model = AclLiteModel(self._model_path)
    
        return constants.SUCCESS

    def pre_process(self, image):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        crop_and_paste_image = \
            self._dvpp.crop_and_paste(yuv_image, image.width, image.height, self._model_width, self._model_height)
        return crop_and_paste_image

    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    def post_process(self, image, infer_output, image_file):
        """
        Post-processing, analysis of inference results
        """
        orig = cv2.imread(image_file, 1)
        w = int(MODEL_HEIGHT * image.width / image.height)
        h = MODEL_HEIGHT
        orig = cv2.resize(orig, (w, h))
        data = infer_output[0]
        vals = data.flatten()
        res = np.sum(vals, axis=0)
        result = round(res / 1000.0)
        data_2 = data.reshape(800, 1408)
        heatMap = data_2[:h, :w]
        heatMap = heatMap.astype(np.uint8)
        heatMap = cv2.GaussianBlur(heatMap, (0, 0), 5, 5, cv2.BORDER_DEFAULT)
        cv2.normalize(heatMap, heatMap, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
        heatMap = cv2.applyColorMap(heatMap, cv2.COLORMAP_JET)
        add_img = cv2.addWeighted(orig, 1, heatMap, 0.5, 0.0)
        cv2.putText(add_img, str(result), (30, 60), cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 8)  
        output_path = os.path.join("../out", os.path.basename(image_file))
        cv2.imwrite(output_path, add_img)
       
SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/count_person.caffe.om")
MODEL_WIDTH = 1408
MODEL_HEIGHT = 800

def main():    
    """
    main
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    acl_resource = AclLiteResource()
    acl_resource.init()

    crowdcount = CrowdCount(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)   
    ret = crowdcount.init()
    
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))
        
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in constants.IMG_EXT]

    for image_file in images_list:        
        image = AclLiteImage(image_file)            
        crop_and_paste_image = crowdcount.pre_process(image)
        print("pre process end")
        result = crowdcount.inference([crop_and_paste_image])              
        result_img_encode = crowdcount.post_process(crop_and_paste_image, result, image_file)      
    return result_img_encode

if __name__ == '__main__':
    main()



