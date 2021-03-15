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
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/atlas_utils"))

from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acl_model import Model
from acl_image import AclImage
from acl_resource import AclResource
from image_net_classes import get_image_net_class
from PIL import Image, ImageDraw, ImageFont

class Classify(object):
    """
    Classify
    """
    def __init__(self, acl_resource, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._model = Model(model_path)

    def pre_process(self, image):
        """
        pre_process
        """    
        bgr_img = cv2.imread(image).astype(np.float32)
        bgr_img = bgr_img / 255.0
        resized_image = cv2.resize(bgr_img,(299,299))
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        """
        inference
        """
        return self._model.execute([resized_image, ])

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
            output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype(os.path.join(SRC_PATH, "SourceHanSansCN-Normal.ttf"), size=30)
            draw.text((10, 50), object_class, font=font, fill=255)
            draw.text((10, 100), str(object_value), font=font, fill=255)
            origin_img.save(output_path)
    


SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/frozen_graph-inception-resnet-test1.om")

MODEL_WIDTH = 299
MODEL_HEIGHT = 299


def main():

    """
    Program execution with picture directory parameters
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclResource()
    acl_resource.init()
    #Instantiation classification detection, incoming om model path, model input width and height parameters
    classify = Classify(acl_resource, MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    #Get the picture storage directory from the parameters, and infer picture by picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    #Create a directory and save the infer results
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))

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
 
