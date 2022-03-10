import cv2 as cv
import numpy as np
import os
import sys
import time
import json
path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from constants import IMG_EXT
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
import acllite_utils as utils
import draw_predict

MODEL_WIDTH = 513
MODEL_HEIGHT = 513
out_w = 56
out_h = 56
INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
model_path = '../model/deeplab_origin.om'
OUTPUT_PATH =  os.path.join(path, '../output/')

@utils.display_time
def preprocess(picPath):
    bgr_img_ = cv.imread(picPath).astype(np.uint8)

    img = cv.resize(bgr_img_, (MODEL_WIDTH, MODEL_HEIGHT))
    img=img.astype(np.float32, copy=False)

    img[:, :, 0] -= 104
    img[:, :, 0] = img[:, :, 0] / 57.375
    img[:, :, 1] -= 117
    img[:, :, 1] = img[:, :, 1] / 57.120
    img[:, :, 2] -= 123
    img[:, :, 2] = img[:, :, 2] / 58.395

    img = img.transpose([2, 0, 1]).copy()

    return img

@utils.display_time
def postprocess(result_list, pic, output_dir):
    draw_predict.draw_label(pic, result_list[0].squeeze(), output_dir)

@utils.display_time
def inference(model, input_data):
    return model.execute(input_data)

@utils.display_time
def main():

    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    acl_resource = AclLiteResource()
    acl_resource.init()
    
    model = AclLiteModel(model_path)
    images_list = [os.path.join(INPUT_DIR, img)
                   for img in os.listdir(INPUT_DIR)
                   if os.path.splitext(img)[1] in IMG_EXT]
    
    print(images_list)
    for pic in images_list:

        l_data = preprocess(pic)
        result_list = inference(model, [l_data,])
        postprocess(result_list, pic, OUTPUT_DIR)

    print("Execute end")


if __name__ == '__main__':
    main()
