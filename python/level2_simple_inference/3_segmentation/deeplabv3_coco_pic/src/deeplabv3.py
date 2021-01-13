import cv2 as cv
import numpy as np
import os
import time
from constants import *
from acl_model import Model
from acl_resource import AclResource


MODEL_WIDTH = 513
MODEL_HEIGHT = 513
INPUT_DIR = './data/'
OUTPUT_DIR = './out/'
MODEL_PATH = './model/deeplabv3_plus.om'

def preprocess(picPath):
    #read img
    bgr_img = cv.imread(picPath)
    print(bgr_img.shape)

    #get img shape
    orig_shape = bgr_img.shape[:2]

    #resize img
    img = cv.resize(bgr_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.int8)

    # save memory C_CONTIGUOUS mode
    if not img.flags['C_CONTIGUOUS']:
        img = np.ascontiguousarray(img)
    return orig_shape,  img


def postprocess(result_list, pic, orig_shape,pic_path):
    result_img = result_list[0].reshape(513,513)
    result_img =result_img.astype('uint8')
    orig_img = cv.imread(pic_path)
    img5 = cv.merge((result_img,result_img,result_img))
    bgr_img = cv.resize(img5,(orig_shape[1],orig_shape[0]))
    print(bgr_img)
    bgr_img = (bgr_img + 255)
    output_pic = os.path.join(OUTPUT_DIR, "out_" + pic)
    cv.imwrite(output_pic, bgr_img)

def main():
    #create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    #acl init
    acl_resource = AclResource()
    acl_resource.init()

    #load model
    model = Model(acl_resource, MODEL_PATH)
    src_dir = os.listdir(INPUT_DIR)

    #infer picture
    for pic in src_dir:
        #read picture
        pic_path = os.path.join(INPUT_DIR, pic)

        #get pic data
        orig_shape,  l_data = preprocess(pic_path)

        #inference
        result_list = model.execute([l_data])    

        #postprocess
        postprocess(result_list, pic, orig_shape,pic_path)
    print("Execute end")


if __name__ == '__main__':
    main()
