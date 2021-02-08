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
from PIL import Image, ImageDraw, ImageFont
import struct

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))

import acl
import atlas_utils.utils as utils
import atlas_utils.constants as constants
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from acl_resource import AclResource

currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'outputs/')
MODEL_PATH = os.path.join(currentPath, "model/deploy_vel.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 224


class Hpa(object):
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

        return constants.SUCCESS

    def pre_process(self, im):
        """
        image preprocess
        """
        self._img_width = im.size[0]
        self._img_height = im.size[1]
        im = im.resize((224, 224))
        # hwc
        img = np.array(im)
        # rgb to bgr
        img = img[:, :, ::-1]
        img = img.astype("float16")
        result = img.transpose([2, 0, 1]).copy()
        return result 

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

    def visualize(self, file_name, pred):    
        """
        visualize
        """

    # 1, ID and name corresponding
        id_2_label = [
        "Mitochondria", "Nucleus", "Endoplasmic reticulum", "Nuclear speckles", 
        "Plasma membrane", "Nucleoplasm", "Cytosol", "Nucleoli",
        "Vesicles", "Golgi apparatus"
    ]
 
    # 2. Read pictures
        setFont = ImageFont.truetype('./font.ttf', 20)
        fillColor = "#fff"
        im = Image.open(file_name)
        im = im.resize((512, 512))
        draw = ImageDraw.Draw(im)
        pred = pred.flatten() 
        top1 = np.argmax(pred)
        print(id_2_label[top1], pred[top1])
        label =  "%s : %.2f%%" % (id_2_label[top1], float(pred[top1]) * 100)
        pred[top1] = 0
        draw.text(xy = (20, 20), text = label, font=setFont, fill=fillColor)

        top2 = np.argmax(pred)
        print(top2, pred.shape)
        label =  "%s : %.2f%%" % (id_2_label[top2], float(pred[top2]) * 100)
        pred[top2] = 0
        draw.text(xy = (20, 50), text = label, font=setFont, fill=fillColor)

        top3 = np.argmax(pred)
        label =  "%s : %.2f%%" % (id_2_label[top3], float(pred[top3]) * 100)
        pred[top3] = 0
        draw.text(xy = (20, 80), text = label, font=setFont, fill=fillColor)
 
    # save photo
        im.save("../outputs/out_" + os.path.basename(file_name))  

    def post_process(self, result, image_name):  
        """
        post_process
        """   
        
        score = np.array(result[0])
        pred = self.sigmoid(score)

        # visualize
        self.visualize(image_name, pred)


def main():
    """
    main
    """
    image_dir = os.path.join(currentPath, "data")

    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in constants.IMG_EXT]

    acl_resource = AclResource()
    acl_resource.init()

    hpa = Hpa(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = hpa.init()
    utils.check_ret("hpa init ", ret)

    # Create a directory to save inference results
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    for image_file in images_list:
        image_name = os.path.join(image_dir, os.path.basename(image_file))
        print('====' + image_name + '====')

        # read image
        im = Image.open(image_name)
        if len(im.split()) != 3:
            print('warning: "{}" is not a color image and will be ignored'.format(image_name))
            continue

        # Preprocess the picture 
        resized_image = hpa.pre_process(im)

        # Inferencecd 
        result = hpa.inference([resized_image, ])

        # # Post-processing
        hpa.post_process(result, image_name)
         

if __name__ == '__main__':
    main()
