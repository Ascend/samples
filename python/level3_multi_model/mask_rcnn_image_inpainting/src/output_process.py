"""post process for 310 inference"""
import argparse
import os
import sys
import numpy as np
from PIL import Image
import cv2
import draw_predict
from config import config

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))

def get_img_size(file_name):
    img = Image.open(file_name)
    return img.size

def get_resize_ratio(img_size):
    org_width, org_height = img_size
    resize_width_ratio = config.img_width / org_width
    if resize_width_ratio > config.img_height / org_height:
        resize_width_ratio = config.img_height / org_height
    
    resize_height_ratio = resize_width_ratio
    
    return resize_width_ratio, resize_height_ratio

def get_eval_result(result_list, img, coordinate, output_path):
    """ Get metrics result according to the annotation file and result file"""
    max_num = 128
    
    img_size = get_img_size(img)
    resize_width_ratio, resize_height_ratio = get_resize_ratio(img_size)

    img_metas = np.array([img_size[1], img_size[0]] + [resize_width_ratio, resize_height_ratio])

    bbox_result = np.array(result_list[0])
    label_result = np.array(result_list[1])
    mask_result = np.array(result_list[2])
    mask_fb_result = np.array(result_list[3])
    all_bbox = bbox_result.astype(np.float16).reshape(80000, 5)
    all_label = label_result.astype(np.int32).reshape(80000, 1)
    all_mask = mask_result.astype(np.bool_).reshape(80000, 1)
    all_mask_fb = mask_fb_result.astype(np.float16).reshape(80000, 28, 28)

    all_bbox_squee = np.squeeze(all_bbox)
    all_label_squee = np.squeeze(all_label)
    all_mask_squee = np.squeeze(all_mask)
    all_mask_fb_squee = np.squeeze(all_mask_fb)

    all_bboxes_tmp_mask = all_bbox_squee[all_mask_squee, :]
    all_labels_tmp_mask = all_label_squee[all_mask_squee]
    all_mask_fb_tmp_mask = all_mask_fb_squee[all_mask_squee, :, :]
    
    all_bboxes_draw_mask = all_bboxes_tmp_mask.copy()
    all_mask_draw_fb_tmp_mask = all_mask_fb_tmp_mask.copy()
    if all_bboxes_tmp_mask.shape[0] > max_num:
        inds = np.argsort(-all_bboxes_tmp_mask[:, -1])
        inds = inds[:max_num]
        all_bboxes_tmp_mask = all_bboxes_tmp_mask[inds]
        all_labels_tmp_mask = all_labels_tmp_mask[inds]
        all_mask_fb_tmp_mask = all_mask_fb_tmp_mask[inds]

    ret_org_mask, ret_dilate_mask = draw_predict.draw_label(img, output_path, all_mask_draw_fb_tmp_mask, all_bboxes_draw_mask, 
                                 all_labels_tmp_mask, coordinate, img_metas, config.coco_classes, config.num_classes)

    return ret_org_mask, ret_dilate_mask

