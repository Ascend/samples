import sys
import os
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

import acl

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))

from atlas_utils.acl_resource import AclResource
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
import atlas_utils.utils as utils

currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'outputs/')
MODEL_PATH = os.path.join(currentPath,"model/vgg16_cat_dog.om")
MODEL_WIDTH = 224
MODEL_HEIGHT = 256
CLS = ['dog', 'cat']


class Classify(object):
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
        self._dvpp = None

    def init(self):
        """
        Initialize
        """
        self._dvpp = Dvpp()
        # Load model
        self._model = Model(self._model_path)

        return const.SUCCESS

    @utils.display_time
    def pre_process(self, image):
        """
        preprocess 
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
    def post_process(self, infer_output, image_file):
        """
        Post-processing, analysis of inference results
        """
        output_path = os.path.join(OUTPUT_DIR, os.path.basename(image_file))
        infer_result = infer_output[0]
        vals = infer_result.flatten()
        pre_index = vals.argsort()[-1]
        
        origin_img = Image.open(image_file)
        draw = ImageDraw.Draw(origin_img)
        font = ImageFont.truetype("./SourceHanSansCN-Normal.ttf", size=50)
        draw.text((10, 50), CLS[pre_index], font=font, fill=255)
        origin_img.save(output_path)
                

def main():
    """
    main
    """
    image_dir = os.path.join(currentPath, "data" )

    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    acl_resource = AclResource()
    acl_resource.init()

    classify = Classify(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = classify.init()
    utils.check_ret("Classify init ", ret)

    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    for image_file in images_list:
        print('=== ' + os.path.basename(image_file) + '===')

        # read image
        image = AclImage(image_file)

        # Preprocess the picture 
        resized_image = classify.pre_process(image)

        # Inferencecd 
        result = classify.inference([resized_image, ])

        # # Post-processing
        classify.post_process(result, image_file)
         

if __name__ == '__main__':
    main()
 
