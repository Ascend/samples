"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
import sys
import cv2
import numpy as np
import os
import time

path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))

import atlas_utils.utils as utils
import atlas_utils.constants as constants
from atlas_utils.acl_model import Model
from atlas_utils.acl_resource import AclResource

out_w = 512
out_h = 512

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
INPUT_DIR = os.path.join(SRC_PATH, '../data/')
OUTPUT_DIR = os.path.join(SRC_PATH, '../output/')
model_path = os.path.join(SRC_PATH, "../model/image_HDR_enhance.om")


def pre_process(dir_name):
    """
    Pre Process
    """
    BGR = cv2.imread(dir_name).astype(np.float32)
    h = BGR.shape[0]
    w = BGR.shape[1]

    BGR = BGR / 255.0
    BGR = cv2.resize(BGR, (out_h, out_w))
    RGB = cv2.cvtColor(BGR, cv2.COLOR_BGR2RGB)
    return RGB, h, w


def post_process(result_list, pic, o_h, o_w):
    """
    Post Process
    """
    data = result_list[0].reshape(out_h, out_w, 3)
    output = (cv2.resize(data, (o_w, o_h)) * 255.0).astype(np.uint8)

    BGR_U8 = cv2.cvtColor(output, cv2.COLOR_RGB2BGR)
    file_name = os.path.join(OUTPUT_DIR, pic)
    cv2.imwrite(file_name, BGR_U8)


def main():
    """
    Program execution
    """
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    acl_resource = AclResource()  # acl intialize
    acl_resource.init()

    model = Model(model_path)  # load model

    src_dir = os.listdir(INPUT_DIR)
    for pic in src_dir:
        if not pic.lower().endswith(('.bmp', '.dib', '.png', '.jpg', '.jpeg', '.pbm', '.pgm', '.ppm', '.tif', '.tiff')):
            print('it is not a picture, %s, ignore this file and continue,' % pic)
            continue
        pic_path = os.path.join(INPUT_DIR, pic)
        RGB_image, o_h, o_w = pre_process(pic_path)  # preprocess

        start_time = time.time()
        result_list = model.execute([RGB_image, ])  # inferring
        end_time = time.time()
        print('pic:{}'.format(pic))
        print('pic_size:{}x{}'.format(o_h, o_w))
        print('time:{}ms'.format(int((end_time - start_time) * 1000)))
        print('\n')
        post_process(result_list, pic, o_h, o_w)  # postprocess
    print('task over')


if __name__ == '__main__':
    main()
