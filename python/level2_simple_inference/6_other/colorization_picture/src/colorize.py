import cv2 as cv
import numpy as np
import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/atlas_utils"))

from constants import IMG_EXT
from acl_model import Model
from acl_image import AclImage
from acl_resource import AclResource


MODEL_WIDTH = 224
MODEL_HEIGHT = 224
out_w = 56
out_h = 56
INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
model_path = '../model/colorization_yuv.om'


def preprocess(picPath):
    
    bgr_img = cv.imread(picPath).astype(np.float32)
    
    orig_shape = bgr_img.shape[:2]
        
    bgr_img = bgr_img / 255.0
    
    lab_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2Lab)
    
    orig_l = lab_img[:,:,0]
    
    if not orig_l.flags['C_CONTIGUOUS']:
        orig_l = np.ascontiguousarray(orig_l)
        
    lab_img = cv.resize(lab_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
      
    l_data = lab_img[:,:,0]
    if not l_data.flags['C_CONTIGUOUS']:
        l_data = np.ascontiguousarray(l_data)
        
    l_data = l_data - 50
    return orig_shape, orig_l, l_data


def postprocess(result_list, pic, orig_shape, orig_l):
    
    result_list[0] = result_list[0].reshape(1,2,56,56).transpose(0,2,3,1)
    result_array = result_list[0][0]
        
    ab_data = cv.resize(result_array, orig_shape[::-1])
       
    result_lab = np.concatenate((orig_l[:, :, np.newaxis], ab_data), axis=2)
    
    result_bgr = (255 * np.clip(cv.cvtColor(result_lab, cv.COLOR_Lab2BGR), 0, 1)).astype('uint8')
      
    file_name = os.path.join(OUTPUT_DIR, "out_" + os.path.basename(pic))
    cv.imwrite(file_name, result_bgr)


def main():
    """
    acl resource initialization
    """
    
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
     #ACL resource initialization    
    acl_resource = AclResource()
    acl_resource.init()
    
    model = Model(model_path)
    images_list = [os.path.join(INPUT_DIR, img)
                   for img in os.listdir(INPUT_DIR)
                   if os.path.splitext(img)[1] in IMG_EXT]

    for pic in images_list:
               
        orig_shape, orig_l, l_data = preprocess(pic)
        result_list = model.execute([l_data,])
        postprocess(result_list, pic, orig_shape, orig_l)
        break
    print("Execute end")


if __name__ == '__main__':
    main()
