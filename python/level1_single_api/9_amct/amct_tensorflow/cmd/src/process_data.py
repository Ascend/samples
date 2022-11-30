#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

data process script.

"""
import os
import glob
import numpy as np
import cv2 # pylint: disable=E0401

PATH = os.path.realpath('./')
IMAGE_PATH = os.path.join(PATH, './data/image/*.jpg')
BIN_PATH = os.path.join(PATH, './data/calibration')
BIN_FILE = os.path.join(BIN_PATH, 'calibration.bin')
CHANNEL_MEANS = [123.68, 116.78, 103.94]  # (R, G, B)
INPUT_SHAPE = (224, 224, 3)  # (height, width, channel)
CALIBRATION_SIZE = 32


def image_preprocces(image_path):
    '''
    将jpeg图像，加工为可以进行推理的数据224*224*3。
    处理步骤：1、图像解码、2、图像resize 3、图像crop（Central Crop） 3、图像归一化 4、image2tensor
    说明：图像预处理采用opencv，本例不代表最优处理方式，请根据模型实际的数据预处理逻辑进行处理，样例仅供参考
    :return:
    '''
    fpath = glob.glob(image_path)
    image_list = []
    count = 0
    for image_file in fpath:
        bgr_img = cv2.imread(image_file)
        if len(bgr_img.shape) != 3:
            continue
        # 默认为BGR，转换为RGB
        img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32)
        img = image_resize(img, 256)
        img = central_crop(img)
        img = mean_normalize(img)
        image_list.append(img)

        count = count + 1
        if count == CALIBRATION_SIZE:
            break
    image_array = np.array(image_list, dtype=np.float32)
    return image_array


def image_resize(image, target_size):
    '''
    可以先采样到256
    :param image:
    :param target_size:
    :return:
    '''
    height, width, _ = np.shape(image)
    if target_size is None:
        target_size = 256

    if height < width:
        size_ratio = target_size / height
    else:
        size_ratio = target_size / width
    resize_shape = (int(width * size_ratio), int(height * size_ratio))
    return cv2.resize(image, resize_shape)


def mean_normalize(image):
    '''
    对单张图片做减均值
    :param image:
    :return:
    '''
    return image - CHANNEL_MEANS


def central_crop(image):
    '''
    从目标图像中心crop 224*224的图像
    :param image:
    :return:
    '''
    height, width, _ = np.shape(image)
    target_h, target_w, _ = INPUT_SHAPE
    amount_to_be_cropped_h = (height - target_h)
    amount_to_be_cropped_w = (width - target_w)
    crop_y = amount_to_be_cropped_h // 2
    crop_x = amount_to_be_cropped_w // 2
    return image[crop_y: crop_y + target_h, crop_x: crop_x + target_h, :]


def main():
    '''
    预处理图片，并将其存为bin文件
    '''
    images = image_preprocces(IMAGE_PATH)
    if not os.path.exists(BIN_PATH):
        os.mkdir(BIN_PATH)
    images.tofile(BIN_FILE)


if __name__ == '__main__':
    main()
