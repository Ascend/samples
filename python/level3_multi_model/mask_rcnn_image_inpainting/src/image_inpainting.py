"""
image inpainting
"""
import sys
import os
import numpy as np
import cv2
from PIL import Image
import glob
import time
import collections

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../common/"))
sys.path.append(os.path.join(path, "../../../common/acllite"))

from acllite_model import AclLiteModel
from acllite_utils import display_time  

currentPath= os.path.abspath(os.path.dirname(path) + os.path.sep + ".")

MODEL_PATH = os.path.join(currentPath, "model/hifill.om")
MODEL_MATMUL_PATH =os.path.join(currentPath, 
     "model/0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648/0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648.om")
INPUT_SIZE = 512  
ATTENTION_SIZE = 32 
MULTIPLE = 6
NPTYPE_FLOAT32 = np.float32

def sort(str_lst):
    """sort"""
    return [s for s in sorted(str_lst)]

def reconstruct_residual_from_patches(residual, multiple):
    """
    reconstruct residual from patches
    """
    residual = np.reshape(residual, [ATTENTION_SIZE, ATTENTION_SIZE, multiple, multiple, 3])
    residual = np.transpose(residual, [0, 2, 1, 3, 4])
    return np.reshape(residual, [ATTENTION_SIZE * multiple, ATTENTION_SIZE * multiple, 3])

def extract_image_patches(img, multiple):
    """
    extract image patch
    """
    h, w, c = img.shape
    img = np.reshape(img, [h // multiple, multiple, w // multiple, multiple, c])
    img = np.transpose(img, [0, 2, 1, 3, 4])
    return img

@display_time
def pre_process(raw_img, raw_mask): 
    """
    process raw image & mask
    """
    raw_mask = raw_mask.astype(NPTYPE_FLOAT32) / 255.
    raw_img = raw_img.astype(NPTYPE_FLOAT32)

    # resize raw image & mask to desinated size
    large_img = cv2.resize(raw_img, (MULTIPLE * INPUT_SIZE, MULTIPLE * INPUT_SIZE), 
                           interpolation = cv2.INTER_LINEAR)
    large_mask = cv2.resize(raw_mask, (MULTIPLE * INPUT_SIZE, MULTIPLE * INPUT_SIZE), 
                            interpolation = cv2.INTER_NEAREST)
    
    img_512 = cv2.resize(raw_img, (INPUT_SIZE, INPUT_SIZE), interpolation = cv2.INTER_NEAREST)
    small_mask = cv2.resize(raw_mask, (INPUT_SIZE, INPUT_SIZE), interpolation = cv2.INTER_NEAREST)
    
    # set hole region to 1. and backgroun to 0.
    small_mask = 1. - small_mask

    mask_512_hwc = small_mask[:, :, 0:1]
    mask_512_chw = mask_512_hwc.transpose(2, 0, 1).copy()
    
    pre_process_img = collections.OrderedDict()
    pre_process_img["large_img"] = large_img
    pre_process_img["large_mask"] = large_mask
    pre_process_img["img_512"] = img_512
    pre_process_img["mask_512_chw"] = mask_512_chw
    return pre_process_img

def get_imgs_masks_file_list(images, masks):
    """get_imgs_masks_file_list"""
    paths_img = glob.glob(images + '/*.*[gG]')
    paths_mask = glob.glob(masks + '/*.*[gG]')
    paths_img = sort(paths_img)
    paths_mask = sort(paths_mask)
    return paths_img, paths_mask

@display_time
def matmul_om_large(matmul_model, attention, residual):
    """
    matul om large
    """
    attention_reshape = attention.reshape(1024, 1024)
    residual_reshape = residual.reshape(1024, 96 * 96 * 3)
    matmul_ret = matmul_model.execute([attention_reshape, residual_reshape])
    return matmul_ret[0].reshape(ATTENTION_SIZE, ATTENTION_SIZE, 3072 * 9)

def residual_aggregate(model, residual, attention):
    """
    MULTIPLE * INPUT_SIZE//ATTENTION_SIZE = 6*512/32 = 96
    """
    residual = extract_image_patches(residual, MULTIPLE * INPUT_SIZE // ATTENTION_SIZE)
    residual = np.reshape(residual, [1, residual.shape[0] * residual.shape[1], -1])
    residual = matmul_om_large(model, attention, residual)
    residual = reconstruct_residual_from_patches(residual, MULTIPLE * INPUT_SIZE // ATTENTION_SIZE)
    return residual

@display_time
def post_process(model, raw_img, large_img, large_mask, inpainted_512, img_512, mask_512, attention):
    """
    compute the raw residual map
    """
    h, w, c = raw_img.shape
    low_base = cv2.resize(inpainted_512.astype(NPTYPE_FLOAT32), 
    (INPUT_SIZE * MULTIPLE, INPUT_SIZE * MULTIPLE), interpolation = cv2.INTER_LINEAR) 
    low_large = cv2.resize(img_512.astype(NPTYPE_FLOAT32), 
    (INPUT_SIZE * MULTIPLE, INPUT_SIZE * MULTIPLE), interpolation = cv2.INTER_LINEAR)
    residual = (large_img - low_large) * large_mask

    # reconstruct residual map using residual aggregation module
    residual = residual_aggregate(model, residual, attention)

    # compute large inpainted result
    res_large = low_base + residual
    res_large = np.clip(res_large, 0., 255.)

    # resize large inpainted result to raw size
    res_raw = cv2.resize(res_large, (w, h), interpolation = cv2.INTER_LINEAR)
    
    # paste the hole region to the original raw image
    mask = cv2.resize(mask_512.astype(NPTYPE_FLOAT32), (w, h), interpolation = cv2.INTER_LINEAR)
    mask = np.expand_dims(mask, axis=2)
    
    res_raw = res_raw * mask + raw_img * (1. - mask)
    return res_raw.astype(np.uint8)

@display_time
def readimages(img_path, mask_path):    
    """
    readimages
    """
    raw_img = cv2.imread(img_path) 
    raw_mask = cv2.imread(mask_path) 
    return raw_img, raw_mask

@display_time
def inference(model, input_data):
    """
    inference
    """
    resultList = model.execute(input_data)

    inpainted_512 = resultList[0]    
    attention = resultList[1]
    mask_512_new = resultList[2] 

    return inpainted_512, attention, mask_512_new

@display_time
def image_inpaint(image_dirs, masks_dirs, output_dir):    
    """
    image inpainting interface
    """
    #load model
    model = AclLiteModel(MODEL_PATH)
    matmul_om = AclLiteModel(MODEL_MATMUL_PATH)
    
    paths_img, paths_mask = get_imgs_masks_file_list(image_dirs, masks_dirs)
    for i in range(len(paths_img)):
        print('==========')        
        raw_img, raw_mask = readimages(paths_img[i], paths_mask[i])
        pre_process_img = pre_process(raw_img, raw_mask)       

        inpainted_512, attention, mask_512_new = inference(model, 
                                    [pre_process_img["img_512"], pre_process_img["mask_512_chw"], ])

        # post-processing
        res_raw_size = post_process(matmul_om, raw_img, pre_process_img["large_img"], pre_process_img["large_mask"], 
                                    inpainted_512[0], pre_process_img["img_512"], mask_512_new[0], attention[0])
        filename = os.path.join(output_dir, 'outpaint_' + os.path.basename(paths_img[i]))
        cv2.imwrite(filename, res_raw_size)
        
    print("Execute end")
