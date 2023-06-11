"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-02-02 09:44:13
MODIFIED: 2021-02-22 09:44:13
"""
import sys
import os
import numpy as np
from PIL import Image

sys.path.append("../../../common/")
sys.path.append("../")
sys.path.append("../../../common/acllite")
import acl
import acllite_utils as utils
import constants as constants
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource


class SingleImageDehaze(object):
    """
    Class for SingleImageDehaze
    """
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._img_width = 0
        self._img_height = 0
        self._model = None

    @staticmethod
    def sigmoid(x):
        """
        sigmod function
        """
        return 1. / (1 + np.exp(-x))

    def init(self):
        """
        Initialize
        """
        # Load model
        self._model = AclLiteModel(self._model_path)

        return constants.SUCCESS

    def pre_process(self, im):
        """
        image preprocess
        """
        self._img_width = im.size[0]
        self._img_height = im.size[1]
        im = im.resize((512, 512))
        # hwc
        img = np.array(im)
        img = img / 127.5 - 1.
        
        # rgb to bgr
        img = img[:, :, ::-1]
        img = img.astype("float16")
        return img 

    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    def post_process(self, infer_output, image_name):
        """
        Post-processing, analysis of inference results
        """
        np.array(infer_output[0])
        result_image = np.reshape(infer_output[0], (512, 512, 3))
        result_image = result_image[:, :, ::-1]
        result_image = np.clip((result_image + 1.) / 2. * 255., 0, 255).astype(np.uint8)
        result_image = Image.fromarray(result_image)
        result_image = result_image.resize((self._img_width, self._img_height))
        result_image.save('../out/out_' + image_name)


def main():
    """
    main
    """
    src_path = os.path.realpath(__file__).rsplit("/", 1)[0]
    model_path = "../model/deploy_vel.om"
    model_width = 512
    model_height = 512

    # With picture directory parameters during program execution
    if len(sys.argv) != 2:
        print("The App arg is invalid")
        exit(1)

    acl_resource = AclLiteResource()
    acl_resource.init()

    single_image_dehaze = SingleImageDehaze(model_path, model_width, model_height)
    ret = single_image_dehaze.init()
    utils.check_ret("single_image_dehaze init ", ret)

    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in constants.IMG_EXT]

    # Create a directory to save inference results
    if not os.path.isdir(os.path.join(src_path, "../out")):
        os.mkdir(os.path.join(src_path, "../out"))

    for image_file in images_list:
        image_name = image_file.split('/')[-1]

        # read image
        im = Image.open(image_file)
        
        # Preprocess the picture 
        resized_image = single_image_dehaze.pre_process(im)

        # Inference
        result = single_image_dehaze.inference([resized_image, ])

        # # Post-processing
        single_image_dehaze.post_process(result, image_name)
         

if __name__ == '__main__':
    main()
