import cv2 as cv
import numpy as np
import os
import sys
import time
import acl
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))
from constants import NPY_BYTE
from constants import IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource
import acllite_utils as utils
from acllite_imageproc import AclLiteImageProc
import draw_predict

MODEL_WIDTH = 513
MODEL_HEIGHT = 513
INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
model_path = '../model/deeplab_quant.om'
OUTPUT_PATH =  os.path.join(path, '../output/')

@utils.display_time
def preprocess(image, dvpp_):
    
    image_dvpp = image.copy_to_dvpp()
    yuv_image = dvpp_.jpegd(image_dvpp)
    resized_image = dvpp_.resize(yuv_image, MODEL_WIDTH, MODEL_HEIGHT)
    return resized_image

@utils.display_time
def postprocess(result_list, pic, output_dir):
    draw_predict.draw_label(pic, result_list[0].squeeze(), output_dir)
    

@utils.display_time
def inference(model, input_data):
    return model.execute(input_data)



@utils.display_time
def main():
    """
    acl resource initialization
    """
    
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
     #ACL resource initialization
    acl_resource = AclLiteResource()
    acl_resource.init()
    dvpp_ = AclLiteImageProc()
    model = AclLiteModel(model_path)
    images_list = [os.path.join(INPUT_DIR, img)
                   for img in os.listdir(INPUT_DIR)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    print(images_list)
    
    start = time.time()
    for pic in images_list:
        image = AclLiteImage(pic)       

        l_data = preprocess(image, dvpp_)
        result_list = inference(model, [l_data,])
        postprocess(result_list, pic, OUTPUT_DIR)

if __name__ == '__main__':
    main()
