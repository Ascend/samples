import sys
import os
import numpy as np
import cv2

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/acllite/"))

import acl
import acllite_utils as utils
import constants as const
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

class Cartoonization(object):
    """
    class for Cartoonization
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
        self._dvpp = AclLiteImageProc()

        # Load model
        self._model = AclLiteModel(self._model_path)

        return const.SUCCESS
 
    @utils.display_time
    def pre_process(self, image):
        """
        image preprocess
        """
        image_dvpp = image.copy_to_dvpp()
        yuv_image = self._dvpp.jpegd(image_dvpp)
        crop_and_paste_image = self._dvpp.crop_and_paste_get_roi(yuv_image, image.width, image.height, \
                                self._model_width, self._model_height)
        return crop_and_paste_image

    @utils.display_time
    def inference(self, resized_image):
        """
        model inference
        """
        return self._model.execute(resized_image)

    @utils.display_time
    def post_process(self, infer_output, image_file, origin_image):
        """
        post process
        """
        data = ((np.squeeze(infer_output[0]) + 1) * 127.5)
        img = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (origin_image.width, origin_image.height))
        output_path = os.path.join("../out", os.path.basename(image_file))
        cv2.imwrite(output_path, img)

currentPath = os.path.join(path, "..")
MODEL_PATH = os.path.join(currentPath, "model/cartoonization.om")
MODEL_WIDTH = 256
MODEL_HEIGHT = 256

def main():
    # check param
    if (len(sys.argv) != 2):
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
    
    # init
    ret = cartoonization.init()
    utils.check_ret("Cartoonization.init ", ret)
    
    # create dir to save result
    if not os.path.isdir('../out'):
        os.mkdir('../out')

    for image_file in images_list:
        # read image
        image = AclLiteImage(image_file)
        # preprocess
        crop_and_paste_image = cartoonization.pre_process(image)
        print("[Sample] pre process end")
        # inference
        result = cartoonization.inference([crop_and_paste_image, ])
        # postprocess
        cartoonization.post_process(result, image_file, image)

if __name__ == '__main__':
    main()
 
