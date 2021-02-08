"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-01-29 14:04:45
"""

import sys
import os

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))

import acl
import atlas_utils.utils as utils
import atlas_utils.constants as const
from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from acl_resource import AclResource
from gesture_categories import get_gesture_categories
from PIL import Image, ImageDraw, ImageFont

class Gesture(object):
    """
	define gesture class
    """
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = None
        self._model = None
    
    def init(self):
     
        """
        Initialize
        """
        # Initialize dvpp
        self._dvpp = Dvpp()

        # Load model
        self._model = Model(self._model_path)

        return const.SUCCESS

    def pre_process(self, image):
        """
        pre_precess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image, self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image

    def inference(self, resized_image):
        """
	    inference
        """
        return self._model.execute(resized_image)

    def post_process(self, infer_output, image_file):
        """
	    post_process
        """
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        top_k = vals.argsort()[-1:-2:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_gesture_categories(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
       
        if len(top_k):
            object_class = get_gesture_categories(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype(os.path.join(SRC_PATH, "SourceHanSansCN-Normal.ttf"), size=30)
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)


SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/gesture_yuv.om")
MODEL_WIDTH = 256
MODEL_HEIGHT = 224


def main():
    """
    main
    """
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    acl_resource = AclResource()
    acl_resource.init()
    
    gesture = Gesture(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    ret = gesture.init()
    utils.check_ret("Gesture.init ", ret)
    
   
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]
    
    
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))

    for image_file in images_list:
        
        image = AclImage(image_file)
        
        resized_image = gesture.pre_process(image)
        print("pre process end")
        
        result = gesture.inference([resized_image, ])
       
        gesture.post_process(result, image_file)

if __name__ == '__main__':
    main()
 

