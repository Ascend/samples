"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
"""
import sys
import os
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/atlas_utils"))

import cv2 as cv
import numpy as np
import acl
import base64
import atlas_utils.utils as utils
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

MODEL_WIDTH = 513
MODEL_HEIGHT = 513

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
MODEL_PATH = os.path.join(SRC_PATH, "../model/deeplabv3_plus.om")
INPUT_DIR = './data/'
OUTPUT_DIR = './outputs/'

def preprocess(picPath):
    """preprocess"""
    #read img
    bgr_img = cv.imread(picPath)

    #get img shape
    orig_shape = bgr_img.shape[:2]

    #resize img
    img = cv.resize(bgr_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.int8)

    # save memory C_CONTIGUOUS mode
    if not img.flags['C_CONTIGUOUS']:
        img = np.ascontiguousarray(img)
    return orig_shape, img


def postprocess(result_list, pic, orig_shape, pic_path):
    """postprocess"""
    result_img = result_list[0].reshape(513, 513)
    result_img = result_img.astype('uint8')
    orig_img = cv.imread(pic_path)
    img = cv.merge((result_img, result_img, result_img))
    bgr_img = cv.resize(img, (orig_shape[1], orig_shape[0]))
    bgr_img = (bgr_img + 255)
    
    output_pic = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(pic))
    print(output_pic)
    cv.imwrite(output_pic, bgr_img)


def main():
    """main"""
    #acl init
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclResource()
    acl_resource.init()
    model = Model(MODEL_PATH)
    dvpp = Dvpp(acl_resource)

    #From the parameters of the picture storage directory, reasoning by a picture
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))

    #infer picture
    for pic in images_list:
        #get pic data
        orig_shape, l_data = preprocess(pic)

        #inference
        result_list = model.execute([l_data])    

        #postprocess
        postprocess(result_list, pic, orig_shape, pic)
    print("Execute end")

if __name__ == '__main__':
    main()
