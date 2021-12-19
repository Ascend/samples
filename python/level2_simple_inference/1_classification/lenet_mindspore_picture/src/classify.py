"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-01-29 14:04:45
"""

import sys
import os
import acl
import image_net_classes
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from constants import ACL_MEM_MALLOC_HUGE_FIRST, ACL_MEMCPY_DEVICE_TO_DEVICE, IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
import numpy as np
import cv2 

from PIL import Image, ImageDraw, ImageFont

def preprocess(bgr_img):
    """
    preprocess
    """
    #get img shape
    orig_shape = bgr_img.shape[:2]

    gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    #normalization
    gray_img = gray_img / 255.0
    gray_img = gray_img / 0.3081
    gray_img = gray_img - 1 * 0.1307 / 0.3081
    #resize img
    gray_img = cv2.resize(gray_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
    print(gray_img.shape)
    
    # save memory C_CONTIGUOUS mode
    if not gray_img.flags['C_CONTIGUOUS']:
        gray_img = np.ascontiguousarray(gray_img)

    return orig_shape, gray_img


def postprocess(infer_output, image_file):
    """
    post_process
    """
    print("post process")
    data = infer_output[0]
    vals = data.flatten()
    max_val=np.max(vals)
    vals = np.exp(vals - max_val)
    sum_val = np.sum(vals) 
    vals /= sum_val

    top_k = vals.argsort()[-1:-6:-1]
    print("images:{}".format(image_file))
    print("======== top5 inference results: =============")
    for n in top_k:
        object_class = image_net_classes.get_image_net_class(n)
        print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
	
    (filepath, tempfilename) = os.path.split(image_file)
    (filename, extension) = os.path.splitext(tempfilename)
    output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), filename + ".txt")	
    with open(output_path, "w", encoding="utf-8") as fp:
        fp.write(image_net_classes.get_image_net_class(top_k[0]))
        

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/mnist.om")
INPUT_DIR = os.path.join(SRC_PATH, "../data/")
OUTPUT_DIR = os.path.join(SRC_PATH, "../outputs/")
MODEL_WIDTH = 32
MODEL_HEIGHT = 32


def main():

    """
    main
    """
    #create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    #acl init
    acl_resource = AclLiteResource()
    acl_resource.init()

    #load model
    model = AclLiteModel(MODEL_PATH)
    src_dir = os.listdir(INPUT_DIR)
    #infer picture
    for pic in src_dir:
        if not pic.lower().endswith(('.bmp', '.dib', '.png', '.jpg', 
                                    '.jpeg', '.pbm', '.pgm', '.ppm', '.tif', '.tiff')):
            print('it is not a picture, %s, ignore this file and continue,' % pic)
            continue
  
        #read picture
        pic_path = os.path.join(INPUT_DIR, pic)
        bgr_img = cv2.imread(pic_path)

        #get pic data
        orig_shape, test_img = preprocess(bgr_img)

        #inference
        result_list = model.execute([test_img, ])    

        #postprocess
        postprocess(result_list, pic)

if __name__ == '__main__':
    main()
 
