#!/usr/bin/env python
# encoding: utf-8
import sys
import os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/atlas_utils"))

import numpy as np
import acl
import base64
import atlas_utils.utils as utils
from PIL import Image, ImageDraw, ImageFont
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/garbage_yuv.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224
image_net_classes = [
   "Seashel", "Lighter","Old Mirror", "Broom","Ceramic Bowl", "Toothbrush","Disposable Chopsticks","Dirty Cloth",
     "Newspaper", "Glassware", "Basketball", "Plastic Bottle", "Cardboard","Glass Bottle", "Metalware", "Hats", "Cans", "Paper",
      "Vegetable Leaf","Orange Peel", "Eggshell","Banana Peel",
    "Battery", "Tablet capsules","Fluorescent lamp", "Paint bucket"]


def get_image_net_class(class_id):
    if class_id >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[class_id]

def pre_process(image, dvpp):
    """preprocess"""
    image_input = image.copy_to_dvpp()
    yuv_image = dvpp.jpegd(image_input)

    print("decode jpeg end")
    resized_image = dvpp.resize(yuv_image, 
                    MODEL_WIDTH, MODEL_HEIGHT)

    print("resize yuv end")
    return resized_image

def post_process(infer_output, image_file):
    print("post process")
    data = infer_output[0]
    vals = data.flatten()
    top_k = vals.argsort()[-1:-6:-1]
    object_class = get_image_net_class(top_k[0])
    output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
    origin_image = Image.open(image_file)
    draw = ImageDraw.Draw(origin_image)
    font = ImageFont.load_default()
    font.size =50
    draw.text((10, 50), object_class, font=font, fill=255)
    origin_image.save(output_path)
    object_class = get_image_net_class(top_k[0])        
    return 

def construct_image_info():
    """construct image info"""
    image_info = np.array([MODEL_WIDTH, MODEL_HEIGHT, 
                           MODEL_WIDTH, MODEL_HEIGHT], 
                           dtype = np.float32) 
    return image_info

def main():    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclResource()
    acl_resource.init()
    model = Model(MODEL_PATH)
    dvpp = Dvpp(acl_resource)
            
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    #Create a directory to store the inference results
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))

    image_info = construct_image_info()
    for image_file in images_list:        
        image = AclImage(image_file)            
        resized_image = pre_process(image, dvpp)
        print("pre process end")
              
        result = model.execute([resized_image,])  
        post_process(result, image_file)    

        print("process "+image_file+" end")
if __name__ == '__main__':
    main()



