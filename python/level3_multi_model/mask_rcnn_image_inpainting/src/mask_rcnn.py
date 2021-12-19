"""
MaskRcnn and imageinpainting
"""
import sys
import os

SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
sys.path.append(os.path.join(SRC_PATH, ".."))
sys.path.append(os.path.join(SRC_PATH, "../../../common/"))
sys.path.append(os.path.join(SRC_PATH, "../../../common/acllite"))

import cv2
import mmcv
import numpy as np
import constants as const
from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource
import output_process
from config import config
import image_inpainting
from PIL import Image

MODEL_PATH = os.path.join(SRC_PATH, "../model/maskrcnn_mindspore_rgb.om")
DATA_PATH = os.path.join(SRC_PATH, '../data/')
MASK_PATH =  os.path.join(SRC_PATH, '../mask/')
OUTPUT_PATH =  os.path.join(SRC_PATH, '../output/')

def get_img_metas(file_name):
    """get_img_metas"""
    img = Image.open(file_name)
    img_size = img.size

    org_width, org_height = img_size
    resize_ratio = config.img_width / org_width
    if resize_ratio > config.img_height / org_height:
        resize_ratio = config.img_height / org_height

    img_metas = np.array([img_size[1], img_size[0]] +
                         [resize_ratio, resize_ratio])
    return img_metas

def process_img(img_file):
    """process_img"""
    img = cv2.imread(img_file)
    #get img shape
    orig_shape = img.shape[:2]
    model_img = mmcv.imrescale(img, (config.img_width, config.img_height))
    if model_img.shape[0] > config.img_height:
        model_img = mmcv.imrescale(model_img,
                                   (config.img_height, config.img_height))
    pad_img = np.zeros(
        (config.img_height, config.img_width, 3)).astype(model_img.dtype)
    pad_img[0:model_img.shape[0], 0:model_img.shape[1], :] = model_img
    pad_img.astype(np.float16)
    return orig_shape, pad_img

def preprocess(picPath):
    """preprocess"""
    orig_shape, pad_img = process_img(picPath)

    # save memory C_CONTIGUOUS mode
    if not pad_img.flags['C_CONTIGUOUS']:
        pad_img = np.ascontiguousarray(pad_img)
        
    img_metas = get_img_metas(picPath).astype(np.float16)

    return orig_shape, pad_img, img_metas

def postprocess(result_list, pic, coordinate, output_path):
    """postprocess"""
    org_mask, dilate_mask = output_process.get_eval_result(result_list, pic, coordinate, output_path)
    
    if not np.all(dilate_mask == 0):
        pic_name = os.path.basename(pic)
        dilate_mask_pic = os.path.join(MASK_PATH, pic_name)
        print(dilate_mask_pic)
        cv2.imwrite(dilate_mask_pic, org_mask) 
        image_inpainting.image_inpaint(DATA_PATH, MASK_PATH, output_path)

def main():
    """main"""
    #acl init
    if (len(sys.argv) != 3):
        print("The App arg is invalid")
        exit(1)
    acl_resource = AclLiteResource()
    acl_resource.init()
    model = AclLiteModel(MODEL_PATH)
    #x=296
    #y=330
    #x=410
    #y=664
    coordinate = [-1, -1]

    #From the parameters of the picture storage directory, reasoning by a picture
    coordinate = [int(sys.argv[1]), int(sys.argv[2])]

    if not os.path.exists(DATA_PATH):
        os.mkdir(DATA_PATH)

    if not os.path.exists(MASK_PATH):
        os.mkdir(MASK_PATH)

    if not os.path.exists(OUTPUT_PATH):
        os.mkdir(OUTPUT_PATH)

    images_list = [os.path.join(DATA_PATH, img)
                   for img in os.listdir(DATA_PATH)
                   if os.path.splitext(img)[1] in const.IMG_EXT]
    #infer picture
    for pic in images_list:
        #get pic data
        orig_shape, l_data, im_info = preprocess(pic)
        #inference
        result_list = model.execute([l_data, im_info])
        #postprocess
        postprocess(result_list, pic, coordinate, OUTPUT_PATH)
    print("Execute end")

if __name__ == '__main__':
    main()
