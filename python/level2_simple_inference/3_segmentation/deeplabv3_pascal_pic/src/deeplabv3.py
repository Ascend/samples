"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
"""
import cv2 as cv
import numpy as np
import os
import time
import constants
import acl_resource
import acl_model

MODEL_WIDTH = 513
MODEL_HEIGHT = 513
INPUT_DIR = './data/'
OUTPUT_DIR = './outputs/'
MODEL_PATH = './model/deeplabv3_plus.om'

def preprocess(picPath):
    """preprocess"""
    #read img
    bgr_img = cv.imread(picPath)
    print(bgr_img.shape)

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
    output_pic = os.path.join(OUTPUT_DIR, "out_" + pic)
    cv.imwrite(output_pic, bgr_img)


def main():
    """main"""
    #create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    #acl init
    aclresource = acl_resource.AclResource()
    aclresource.init()

    #load model
    model = acl_model.Model(aclresource, MODEL_PATH)
    src_dir = os.listdir(INPUT_DIR)

    #infer picture
    for pic in src_dir:
        #read picture
        pic_path = os.path.join(INPUT_DIR, pic)

        #get pic data
        orig_shape, l_data = preprocess(pic_path)

        #inference
        result_list = model.execute([l_data])    

        #postprocess
        postprocess(result_list, pic, orig_shape, pic_path)
    print("Execute end")

if __name__ == '__main__':
    main()
