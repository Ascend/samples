"""main"""
import sys
sys.path.append("../../../../common")
sys.path.append("../")
import os
import time
import numpy as np
import cv2 as cv
import acl
import atlas_utils.utils as utils
from PIL import Image, ImageDraw, ImageFont
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_resource import AclResource


MODEL_PATH1 = "../model/xingkong1_fp32_nchw_no_aipp.om"
MODEL_PATH2 = "../model/tangguo_fp32_nchw_no_aipp.om"
MODEL_PATH3 = "../model/bijiasuo_fp32_nchw_no_aipp.om"
MODEL_PATH4 = "../model/work_soldiers_fp32_nchw_no_aipp.om"
OUTPUT_DIR = '../outputs/'
MODEL_WIDTH = 1080
MODEL_HEIGHT = 720


def pre_process(picPath):
    """preprocess"""
    bgr_img = cv.imread(picPath).astype(np.float32)
    orig_shape = bgr_img.shape[:2]
    print("orig_shape input shape = ", orig_shape)
    rgb_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2RGB)
    rgb_img = cv.resize(rgb_img, (MODEL_WIDTH, MODEL_HEIGHT))
    rgb_img = rgb_img.transpose(2, 0, 1).copy()
    return orig_shape, rgb_img


def post_process(result_list, orig_shape, image_file):
    """postprocess"""
    result = result_list[0]
    result = result.reshape(3, 360, 540).astype(np.float32)
    res_data = result.transpose(1, 2, 0).copy()
    res_data = cv.cvtColor(res_data, cv.COLOR_RGB2BGR)
    res_data = cv.resize(res_data,orig_shape[::-1])
    output_file = os.path.join(OUTPUT_DIR, "out_" + os.path.basename(image_file))
    print("output:%s" % output_file)
    cv.imwrite(output_file, res_data)


def main():
    """
    Program execution with picture directory parameters
    """
    
    if (len(sys.argv) != 3):
        print("The App arg is invalid. The style you can choose: \
                xingkong/tangguo/bijiasuo/worksoldiers.eg: python3 main.py ../data xingkong")
        exit(1)
    
    style_type = sys.argv[2]
    if style_type == "tangguo":
        model_path = '../model/tangguo_fp32_nchw_no_aipp.om'
    elif style_type == "bijiasuo":
        model_path = '../model/bijiasuo_fp32_nchw_no_aipp.om'
    elif style_type == "worksoldiers":
        model_path = '../model/work_soldiers_fp32_nchw_no_aipp.om'
    elif style_type == "xingkong":
        model_path = '../model/xingkong1_fp32_nchw_no_aipp.om'

    acl_resource = AclResource()
    acl_resource.init()
    model = Model(model_path)

    image_dir = sys.argv[1]
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]
    if not os.path.isdir('../outputs'):
        os.mkdir('../outputs')

    for image_file in images_list:
        orig_shape, rgb_data = pre_process(image_file)
        print("pre process end")
        result_list = model.execute([rgb_data])  
        print("Execute end")    
        post_process(result_list, orig_shape, image_file)
        print("postprocess end")    

if __name__ == '__main__':
    main()
 

