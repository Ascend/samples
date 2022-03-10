"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
import sys
import os
import acl
import cv2
import numpy as np
import time
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/acllite"))

from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont
from acllite_utils import display_time 

class Classify(object):
    """
    Classify
    """
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._model = AclLiteModel(model_path)

    @display_time
    def pre_process(self, image):
        """
        pre_process
        """    
        bgr_img = cv2.imread(image).astype(np.float32)
        bgr_img = bgr_img / 255.0
        resized_image = cv2.resize(bgr_img, (299, 299))
        return resized_image
    
    @display_time
    def inference(self, resized_image):
        """
        inference
        """
        return self._model.execute([resized_image, ])

    @display_time
    def post_process(self, infer_output, image_file):
        """
        post_process
        """    
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-7:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_image_net_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        #using pillow, the category with the highest confidence is written on the image and saved locally
        if len(top_k):
            object_class = get_image_net_class(top_k[0])
            object_value = vals[top_k[0]]
            output_path = os.path.join(os.path.join(SRC_PATH, "../out"), os.path.basename(image_file))
            origin_img = cv2.imread(image_file)
            font = cv2.FONT_HERSHEY_SIMPLEX
            origin_img = cv2.putText(origin_img, object_class, (10, 100), font, 3, (255, 255, 255), 3)
            origin_img = cv2.putText(origin_img, str(object_value), (10, 200), font, 2, (255, 255, 255), 3)
            cv2.imwrite(output_path, origin_img)
    
SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/frozen_graph-inception-resnet-test1.om")
MODEL_WIDTH = 299
MODEL_HEIGHT = 299

@display_time
def main():

    """
    Program execution with picture directory parameters
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclLiteResource()
    acl_resource.init()
    #Instantiation classification detection, incoming om model path, model input width and height parameters
    classify = Classify(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #Get the picture storage directory from the parameters, and infer picture by picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #Create a directory and save the infer results
    if not os.path.isdir(os.path.join(SRC_PATH, "../out")):
        os.mkdir(os.path.join(SRC_PATH, "../out"))

    for image_file in images_list:
        #preprocess image
        resized_image = classify.pre_process(image_file)
        print("pre process end")
        #inference
        result = classify.inference(resized_image)
        #post process
        classify.post_process(result, image_file)

if __name__ == '__main__':
    main()
 
