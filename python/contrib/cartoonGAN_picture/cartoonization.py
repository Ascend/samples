import sys
import os
import numpy as np
import cv2

import acl
from utils import *
from acl_dvpp import Dvpp
from acl_model import Model
from acl_image import AclImage



class Cartoonization(object):
    def __init__(self, model_path, model_width, model_height):
        self.device_id = 0
        self.context = None
        self.stream = None
        self._model_path = model_path
        self._model_width = model_width
        self._model_height = model_height
        self._dvpp = None

    def __del__(self):
        if self._model:
            del self._model
        if self._dvpp:
            del self._dvpp
        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("[Sample] class Samle release source success")

    def _init_resource(self):
        print("[Sample] init resource stage:")
        ret = acl.init()
        check_ret("acl.rt.set_device", ret)

        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)

        self.run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)

        print("[Sample] Init resource stage success")

    def init(self):
        # init acl resource
        self._init_resource() 
        self._dvpp = Dvpp(self.stream, self.run_mode)

        # init dvpp
        ret = self._dvpp.init_resource()
        if ret != SUCCESS:
            print("Init dvpp failed")
            return FAILED
        
        # load model
        self._model = Model(self.run_mode, self._model_path)
        ret = self._model.init_resource()
        if ret != SUCCESS:
            print("Init model failed")
            return FAILED
        return SUCCESS

    def pre_process(self, image):
        yuv_image = self._dvpp.jpegd(image)
        crop_and_paste_image = \
            self._dvpp.crop_and_paste(yuv_image, image.width, image.height, self._model_width, self._model_height)
        print("[Sample] crop_and_paste yuv end")
        return crop_and_paste_image

    def inference(self, resized_image):
        return self._model.execute(resized_image.data(), resized_image.size)

    def post_process(self, infer_output, image_file, origin_image):
        print("[Sample] post process")
        data = ((np.squeeze(infer_output[0]) + 1) * 127.5)
        img = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (origin_image.width, origin_image.height))
        output_path = os.path.join("./outputs", os.path.basename(image_file))
        cv2.imwrite(output_path, img)


MODEL_PATH = "./model/cartoonization.om"
MODEL_WIDTH = 256
MODEL_HEIGHT = 256


def main():
    # check param
    if (len(sys.argv) != 2):
        print("The App arg is invalid")
        exit(1)
    
    # instantiation Cartoonization object
    cartoonization = Cartoonization(MODEL_PATH, MODEL_WIDTH, MODEL_HEIGHT)
    # init
    ret = cartoonization.init()
    check_ret("Cartoonization.init ", ret)
    
    # get all pictures
    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    # create dir to save result
    if not os.path.isdir('./outputs'):
        os.mkdir('./outputs')

    for image_file in images_list:
        # read image
        image = AclImage(image_file)
        # preprocess
        crop_and_paste_image = cartoonization.pre_process(image)
        print("[Sample] pre process end")
        # inference
        result = cartoonization.inference(crop_and_paste_image)
        # postprocess
        cartoonization.post_process(result, image_file, image)


if __name__ == '__main__':
    main()
 
