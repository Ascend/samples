import sys
import os
import numpy as np
import cv2

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))

import acl
import acllite_utils as utils
import constants as const
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
from acllite_imageproc import AclLiteImageProc
from time import *

class Cartoonization(object):
    """
    class for Cartoonization
    """
    def __init__(self, model_path, model_width, model_height):
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self.device_id = 0
        self._model = None
        self._dvpp = None

    def init(self):
        """
        Initialize
        """
        self._dvpp = AclLiteImageProc()
        # Load model
        self._model = AclLiteModel(self._model_path)
        
        return const.SUCCESS

    @utils.display_time
    def pre_process(self, image, size=[256, 256]):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        crop_and_paste_image = self._dvpp.crop_and_paste_get_roi(yuv_image, image.width, image.height, \
                                self._model_width, self._model_height)
        return crop_and_paste_image

    def inference(self, resized_image):
        """
        model inference
        """
        return self._model.execute(resized_image)

    @utils.display_time
    def post_process(self, infer_output, image_file):
        """
        post process
        """
        origin_image = cv2.imread(image_file).astype(np.float32)
        h, w = origin_image.shape[:2]

        image = ((np.squeeze(infer_output[0]) + 1) / 2 * 255)
        image = np.clip(image, 0, 255).astype(np.uint8)
        image = cv2.resize(image, (w, h))

        output_path = os.path.join("../out", os.path.basename(image_file))
        cv2.imwrite(output_path, cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
global_shape = sys.argv[2]
currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'out')
MODEL_PATH = os.path.join(currentPath, "model/AnimeGANv2_"+ global_shape +".om")
MODEL_WIDTH = int(global_shape)
MODEL_HEIGHT = int(global_shape)

def main():
    # check param
    if (len(sys.argv) != 3):
        print("The App arg is invalid")
        exit(1)
    # get all pictures
    image_dir = sys.argv[1]
    
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    acl_resource = AclLiteResource()
    acl_resource.init()

    # instantiation Cartoonization object
    cartoonization = Cartoonization(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    ret = cartoonization.init()
    utils.check_ret("Cartoonization.init ", ret)
    
    # create dir to save result
    if not os.path.isdir('../out'):
        os.mkdir('../out')

    for image_file in images_list:
        # preprocess
        image = AclLiteImage(image_file)
        test_img = cartoonization.pre_process(image)
        # inference
        y_pred = cartoonization.inference([test_img, ])
        # postprocess
        cartoonization.post_process(y_pred, image_file)
if __name__ == '__main__':
    main()
 

