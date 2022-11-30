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
import glob
import os
import numpy as np
from PIL import Image # pylint: disable=E0401

PATH = os.path.realpath('./')
IMAGE_PATH = os.path.join(PATH, './data/image/*.jpg')
BIN_PATH = os.path.join(PATH, './data/calibration')
BIN_FILE = os.path.join(BIN_PATH, 'calibration.bin')
CHANNEL_MEANS = [123.68, 116.78, 103.94]  # (R, G, B)
INPUT_SHAPE = (3, 224, 224)  # (channel, height, width)
CALIBRATION_SIZE = 1


def image_preprocces(image_path):
    """
    Processing the image to ResNet-50 inference size in 224*224*3.
    """
    fpath = glob.glob(image_path)
    image_list = []
    count = 0
    for image_file in fpath:
        input_image = Image.open(image_file)
        input_image = input_image.resize((256, 256))
        img = np.array(input_image)
        # crop image
        height = img.shape[0]
        width = img.shape[1]
        h_off = int((height - INPUT_SHAPE[1]) / 2)
        w_off = int((width - INPUT_SHAPE[2]) / 2)
        crop_img = img[h_off:height - h_off, w_off:width - w_off, :]
        img = crop_img[:, :, ::-1]

        # mean_normalize
        img = img.astype("float32")
        img[:, :, 0] -= CHANNEL_MEANS[2]
        img[:, :, 1] -= CHANNEL_MEANS[1]
        img[:, :, 2] -= CHANNEL_MEANS[0]

        image_list.append(img)

        count = count + 1
        if count == CALIBRATION_SIZE:
            break

    # to N,C,H,W
    image_array = np.array(image_list, dtype=np.float32)
    result = image_array.transpose([0, 3, 1, 2])
    return result


def main():
    """
    Images pre-processing and converting to bin file.
    """
    # preprocess data to np
    images = image_preprocces(IMAGE_PATH)
    if not os.path.exists(BIN_PATH):
        os.mkdir(BIN_PATH)
    # save processed data to bin file
    images.tofile(BIN_FILE)


if __name__ == '__main__':
    main()
