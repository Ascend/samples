#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import acl
import numpy as np

import time
from PIL import Image, ImageDraw, ImageFont
from constants import *
from acl_model import Model
from acl_resource import AclResource
import vis

INPUT_DIR = './data/'
OUTPUT_DIR = './out/'
SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/fcn-8s.om")
MODEL_WIDTH = 500
MODEL_HEIGHT = 500

def pre_process( picPath):
    origin_img = Image.open(picPath)
    rgb_img =origin_img.resize((MODEL_WIDTH, MODEL_HEIGHT))
    in_ = np.array(rgb_img, dtype=np.float32) 
    in_ = in_[:,:,::-1] 
    in_ -= np.array((104.00698793, 116.66876762, 122.67891434)) 
    in_ = in_.transpose((2,0,1)).copy()
    return origin_img, in_

def post_process(infer_output, image_file, origin_img):
    print("post process")
    data = infer_output[0]
    data = np.squeeze(data)
    prediction = data.argmax(axis = 0).astype(np.uint8) 
    voc_palette = vis.make_palette(21) 
    out_im = Image.fromarray(vis.color_seg(prediction, voc_palette)) 
    out_im2 =out_im.resize((origin_img.width, origin_img.height))
    output_path = os.path.join(os.path.join(SRC_PATH, "../outputs/"),'out_'+ os.path.basename(image_file))
    output_path2 = os.path.join(os.path.join(SRC_PATH, "../outputs/"),'outvis_'+ os.path.basename(image_file))
    out_im2.save(output_path) 

    in_ = np.array(origin_img, dtype=np.uint8) 
    #print(in_.shape)

    #print(out_im2.shape)
    #masked_im = origin_img *0.5 + out_im2*0.5 #Image.fromarray(vis.vis_seg(origin_img, out_im2, voc_palette)) 
   # masked_im.save(output_path2) 
    return SUCCESS


def main():    
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclResource()
    acl_resource.init()
    
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))
        
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    model = Model(acl_resource, MODEL_PATH)

    for image_file in images_list:                
        origin_img, resized_image = pre_process(image_file)
        print("pre process end")              

        result = model.execute([resized_image,])
        print("Inference  end")              

        result_img_encode = post_process(result, image_file, origin_img)             
        print("post process  end")     
    return result_img_encode

if __name__ == '__main__':
    main()



