#!/usr/bin/env python
# encoding: utf-8
import sys
import os

import acl
import base64

from utils import *
from acl_dvpp import Dvpp
from acl_model import Model
from acl_image import AclImage
from image_net_classes26 import get_image_net_class
from PIL import Image, ImageDraw, ImageFont
import numpy as np


class Classify(object):
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

    def __del__(self):
        if self._model:
            del self._model
        if self._dvpp:
            del self._dvpp
        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("[Sample]  Sample release source success")       

        
    def _init_resource(self):
        print("[Sample] init resource stage:")

        ret = acl.init()
        check_ret("acl.rt.set_device", ret)
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)

        self.run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)
        print("Init resource stage success") 

    def init(self):        
        self._init_resource() 
        self._dvpp = Dvpp(self.stream, self.run_mode)
        
        ret = self._dvpp.init_resource()
        if ret != SUCCESS:
            print("Init dvpp failed")
            return FAILED        
        
        self._model = Model(self.run_mode, self._model_path)
        ret = self._model.init_resource()
        if ret != SUCCESS:
            print("Init model failed")
            return FAILED

        return SUCCESS

    def pre_process(self, image):
        yuv_image = self._dvpp.jpegd(image)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image, 
                        self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        return self._model.execute(resized_image.data(), resized_image.size)

    def post_process(self, infer_output, image_file):
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-6:-1]
        object_class = get_image_net_class(top_k[0])
        output_path = os.path.join(os.path.join(SRC_PATH, "../outputs/"),'out_'+ os.path.basename(image_file))
        origin_img = Image.open(image_file)
        draw = ImageDraw.Draw(origin_img)
        font = ImageFont.load_default()
        font.size =50
        draw.text((10, 50), object_class, font=font, fill=255)
        origin_img.save(output_path)
        object_class = get_image_net_class(top_k[0])        
        return object_class

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/garbage_yuv.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224

def main():    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    classify = Classify(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)   
    ret = classify.init()
    
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))
        
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]

    for image_file in images_list:        
        image = AclImage(image_file)            
        resized_image = classify.pre_process(image)
        print("pre process end")
              
        result = classify.inference(resized_image)              
        result_img_encode = classify.post_process(result, image_file)      
    return result_img_encode

def base64_decode(img_encode, img_type):
    img_np = np.fromstring(base64.b64decode(img_encode), np.uint8)   
    img_file = "./origin/origin." + img_type    
    with open(img_file, 'wb') as f:
        f.write(base64.b64decode(img_encode))
        f.close()    
    return img_np  

if __name__ == '__main__':
    main()



