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

sys.path.append("../../../../common/")
sys.path.append("../")
import acl
import atlas_utils.utils as utils
import atlas_utils.constants as constants
from atlas_utils.acl_dvpp import Dvpp
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from acl_resource import AclResource


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

        return constants.SUCCESS

    def pre_process(self, image):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        print("decode jpeg end")
        resized_image = self._dvpp.resize(yuv_image,
                                          self._model_width, self._model_height)
        print("resize yuv end")
        return resized_image     

    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    def post_process(self, infer_output, image_name):
        """
        get mask
        """
        data = infer_output[0]
        vals = data.flatten()
        mask = np.clip((vals * 255), 0, 255)
        mask = mask.reshape(224, 224, 2)
        cv2.imwrite(os.path.join("../outputs/mask/", image_name), mask[:, :, 0])
        return mask 


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
    cv2.imwrite(os.path.join("../outputs/result/", mask_path.split('/')[-1]), result)


def main():
    """
    main
    """
    SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
    MODEL_PATH = "../model/portrait.om"
    MODEL_WIDTH = 224
    MODEL_HEIGHT = 224

    # With picture directory parameters during program execution
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclResource()
    acl_resource.init()

    seg = Seg(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = seg.init()
    utils.check_ret("seg.init ", ret)

    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in constants.IMG_EXT]

    # Create a directory to save inference results
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs"))
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs/mask")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs/mask"))
    if not os.path.isdir(os.path.join(SRC_PATH, "../outputs/result")):
        os.mkdir(os.path.join(SRC_PATH, "../outputs/result"))

    for image_file in images_list:
        input_data = []
        image_name = image_file.split('/')[-1]
        if image_name != 'background.jpg':
            # read image
            image = AclImage(image_file)
            # Preprocess the picture
            resized_image = seg.pre_process(image)
            input_data.append(resized_image)
            # Inference
            result = seg.inference(input_data)
            # Post-processing
            mask = seg.post_process(result, image_name)
            # Fusion of segmented portrait and background image
            result = background_replace('../data/background.jpg', \
                                        image_file, os.path.join("../outputs/mask/", image_name))
if __name__ == '__main__':
    main()
