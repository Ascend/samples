import sys
import os
import numpy as np
import cv2

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))

import acl
import atlas_utils.utils as utils
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource


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

    def init(self):
        """
        Initialize
        """
        # Load model
        self._model = Model(self._model_path)

        return const.SUCCESS

    @utils.display_time
    def pre_process(self, image_path, size=[256, 256]):
        """
        image preprocess
        """
        img = cv2.imread(image_path).astype(np.float32)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self._model_width, self._model_height))
        img = img / 127.5 - 1.0
        img = np.expand_dims(img, axis=0)

        return img

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

        output_path = os.path.join("../outputs", os.path.basename(image_file))
        cv2.imwrite(output_path, cv2.cvtColor(image, cv2.COLOR_BGR2RGB))


currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'outputs')
MODEL_PATH = os.path.join(currentPath, "model/AnimeGANv2.om")
MODEL_WIDTH = 512
MODEL_HEIGHT = 512

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

    acl_resource = AclResource()
    acl_resource.init()
    
    # instantiation Cartoonization object
    cartoonization = Cartoonization(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    
    # init
    ret = cartoonization.init()
    utils.check_ret("Cartoonization.init ", ret)
    
    # create dir to save result
    if not os.path.isdir('../outputs'):
        os.mkdir('../outputs')

    for image_file in images_list:
        # preprocess
        test_img = cartoonization.pre_process(image_file)
        # inference
        y_pred = cartoonization.inference([test_img, ])

        # postprocess
        cartoonization.post_process(y_pred, image_file)


if __name__ == '__main__':
    main()
 

