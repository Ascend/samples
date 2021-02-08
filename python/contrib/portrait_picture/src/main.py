"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-01-29 14:04:45
"""
import sys
import os
import numpy as np
import cv2

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
OUTPUT_DIR = os.path.join(currentPath, 'outputs/result')
MASK_DIR = os.path.join(currentPath, 'outputs/mask')
MODEL_PATH = os.path.join(currentPath,"model/portrait.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224

class Seg(object):
    """
    Class for portrait segmentation
    """
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self.device_id = 0
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

    @utils.display_time
    def pre_process(self, image):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        resized_image = self._dvpp.resize(yuv_image,
                                          self._model_width, self._model_height)
        return resized_image     

    @utils.display_time
    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    @utils.display_time
    def post_process(self, infer_output, image_name):
        """
        get mask
        """
        data = infer_output[0]
        vals = data.flatten()
        mask = np.clip((vals * 255), 0, 255)
        mask = mask.reshape(224, 224, 2)
        cv2.imwrite(os.path.join(MASK_DIR, image_name), mask[:, :, 0])
        return mask 

@utils.display_time
def background_replace(bg_path, ori_path, mask_path):
    """
    Combine the segmented portrait with the background image
    """
    background = cv2.imread(bg_path)
    height, width = background.shape[:2]
    ori_img = cv2.imread(ori_path)
    mask = cv2.imread(mask_path, 0)
    mask = mask / 255
    mask_resize = cv2.resize(mask, (width, height))
    mask_bg = np.repeat(mask_resize[..., np.newaxis], 3, 2)
    result = np.uint8(background * mask_bg + ori_img * (1 - mask_bg))
    cv2.imwrite(os.path.join(OUTPUT_DIR, os.path.basename(mask_path)), result)


def main():
    """
    main
    """
    image_dir = os.path.join(currentPath, "data" )

    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    if not os.path.exists(MASK_DIR):
        os.mkdir(MASK_DIR)

    acl_resource = AclResource()
    acl_resource.init()

    seg = Seg(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = seg.init()
    utils.check_ret("seg.init ", ret)

    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    for image_file in images_list:
        image_name = os.path.basename(image_file)
        if image_name != 'background.jpg':
            print('====' + image_name + '====')
            # read image
            image = AclImage(image_file)
            # Preprocess the picture
            resized_image = seg.pre_process(image)
            # Inference
            result = seg.inference([resized_image, ])
            # Post-processing
            mask = seg.post_process(result, image_name)
            # Fusion of segmented portrait and background image
            background_replace(os.path.join(image_dir, 'background.jpg'), \
                                        image_file, os.path.join(MASK_DIR, image_name))


if __name__ == '__main__':
    main()
