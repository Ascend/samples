"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
import sys
import os
import numpy as np
import cv2
from PIL import Image
import struct

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))

import acl
import atlas_utils.utils as utils
import atlas_utils.constants as const
from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource

currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'outputs/')
MODEL_PATH = os.path.join(currentPath, "model/rcf.om")
MODEL_WIDTH = 512
MODEL_HEIGHT = 512

class EdgeDetection(object):
    """
    Class for portrait segmentation
    """
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._img_width = 0
        self._img_height = 0
        self._model = None

    def init(self):
        """
        Initialize
        """
        # Load model
        self._model = Model(self._model_path)

        return const.SUCCESS

    @utils.display_time
    def pre_process(self, im):
        """
        image preprocess
        """
        self._img_width = im.size[0]
        self._img_height = im.size[1]
        im = im.resize((512, 512))
        # hwc
        img = np.array(im)
        # rgb to bgr
        img = img[:, :, ::-1]
        img = img.astype("float16")
        result = img.transpose([2, 0, 1]).copy()
        return result 

    @utils.display_time
    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    def sigmoid(self, x):
        """
        sigmod function
        """
        return 1. / (1 + np.exp(-x))

    @utils.display_time
    def post_process(self, infer_output, image_name):
        """
        Post-processing, analysis of inference results
        """
        out_size = [512, 256, 128, 64, 63]
        edge = np.zeros((len(out_size), out_size[0], out_size[0]), dtype=np.float64)
        for idx in range(5):
            result = infer_output[idx]
            img = np.array(result)
            img = np.reshape(img, (out_size[idx], out_size[idx]))
            if idx != 0:
                img = Image.fromarray(img)
                img = img.resize((out_size[0], out_size[0]))
                img = np.array(img)
            edge[idx] = img
        final_edge = 0.2009036 * edge[0] + 0.2101715 * edge[1] + \
                        0.22262956 * edge[2] + 0.22857015 * edge[3] + \
                                0.2479302 * edge[4] + 0.00299916

        final_edge = self.sigmoid(final_edge)
        resultimage = Image.fromarray(np.uint8((1 - final_edge)*255))
        resultimage = resultimage.resize((self._img_width, self._img_height))
        resultimage.save('../outputs/out_' + image_name)


def main():
    """
    main
    """
    image_dir = os.path.join(currentPath, "data")

    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    acl_resource = AclResource()
    acl_resource.init()

    edge_detection = EdgeDetection(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = edge_detection.init()
    utils.check_ret("edge_detection init ", ret)

    # Create a directory to save inference results
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    for image_file in images_list:
        image_name = os.path.basename(image_file)
        print('====' + image_name + '====')

        # read image
        im = Image.open(image_file)
        if len(im.split()) != 3:
            print('warning: "{}" is not a color image and will be ignored'.format(image_file))
            continue

        # Preprocess the picture 
        resized_image = edge_detection.pre_process(im)

        # Inferencecd 
        result = edge_detection.inference([resized_image, ])

        # # Post-processing
        edge_detection.post_process(result, image_name)
         

if __name__ == '__main__':
    main()
