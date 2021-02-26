"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-6-04 20:12:13
MODIFIED: 2020-6-28 14:04:45
"""
import sys
import json
import cv2
import numpy as np
import os
import time
import pickle
sys.path.append("../../../../common/atlas_utils")
sys.path.append("../../../../common")
sys.path.append("../")
from acl_model import Model
from acl_resource import AclResource

MODEL_WIDTH = 416
MODEL_HEIGHT = 416
INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
MODEL_PATH = '../model/yolov4_no_postprocess.om'
CLASS_NUM = 80
NMS_THRESHOLD_CONST = 0.5
CLASS_SCORE_CONST = 0.4
MODEL_OUTPUT_BOXNUM = 10647
labels = ["person",
        "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench",
        "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
        "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
        "pizza", "donut", "cake", "chair", "sofa", "potted plant", "bed", "dining table",
        "toilet", "TV monitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]


def func_nms(boxes, nms_threshold):
    """
    func_nms
    """
    b_x = boxes[:, 0]
    b_y = boxes[:, 1]
    b_w = boxes[:, 2]
    b_h = boxes[:, 3]
    scores = boxes[:, 5]
    areas = (b_w + 1) * (b_h + 1)

    order = scores.argsort()[::-1] 
    keep = []  # keep box 
    while order.size > 0:
        i = order[0]
        keep.append(i)  # keep max score 
        # inter area  : left_top   right_bottom
        xx1 = np.maximum(b_x[i], b_x[order[1:]])
        yy1 = np.maximum(b_y[i], b_y[order[1:]])
        xx2 = np.minimum(b_x[i] + b_w[i], b_x[order[1:]] + b_w[order[1:]])
        yy2 = np.minimum(b_y[i] + b_h[i], b_y[order[1:]] + b_h[order[1:]])

        #inter area
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h

        #union area : area1 + area2 - inter
        union = areas[i] + areas[order[1:]] - inter

        # calc IoU
        IoU = inter / union
        
        inds = np.where(IoU <= nms_threshold)[0]
        order = order[inds + 1]  

    final_boxes = [boxes[i] for i in keep]
    return final_boxes


def preprocess(bgr_img):
    """
    preprocess
    """
    #get img shape
    orig_shape = bgr_img.shape[:2]

    rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
    #normalization
    rgb_img = rgb_img / 255.0
    #resize img
    rgb_img = cv2.resize(rgb_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
    # save memory C_CONTIGUOUS mode
    if not rgb_img.flags['C_CONTIGUOUS']:
        rgb_img = np.ascontiguousarray(bgr_img)

    return orig_shape, rgb_img


def postprocess(result_list, orig_shape, frame, pic):  
    """
    postprocess
    """
    x_scale = orig_shape[1] / MODEL_HEIGHT
    y_scale = orig_shape[0] / MODEL_WIDTH

    result_class = result_list[0].reshape(MODEL_OUTPUT_BOXNUM, 80).astype('float32')
    result_box = result_list[1].reshape(MODEL_OUTPUT_BOXNUM, 4).astype('float32')
    boxes = np.zeros(shape=(MODEL_OUTPUT_BOXNUM, 6), dtype = np.float32)
    boxes[:, :4]= result_box
    list_score = result_class.max(axis = 1)
    list_class = result_class.argmax(axis=1)
    list_score = list_score.reshape(MODEL_OUTPUT_BOXNUM, 1)
    list_class=list_class.reshape(MODEL_OUTPUT_BOXNUM, 1)
    boxes[:, 4]= list_class[:, 0]
    boxes[:, 5]= list_score[:, 0]
    all_boxes = boxes[boxes[:, 5] >= CLASS_SCORE_CONST]
    real_box = func_nms(np.array(all_boxes), NMS_THRESHOLD_CONST)

    for i, detect_result in enumerate(real_box):
        top_x= int((detect_result[0] - detect_result[2] / 2) * x_scale)
        top_y= int((detect_result[1] - detect_result[3] / 2) * y_scale)
        bottom_x= int((detect_result[0] + detect_result[2] / 2) * x_scale)
        bottom_y= int((detect_result[1] + detect_result[3] / 2) * y_scale)
        cv2.rectangle(frame, (top_x, top_y), (bottom_x, bottom_y), (0, 255, 0), 1)
        cv2.putText(frame, labels[int(detect_result[4])], (top_x + 5, top_y + 10), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    output_pic = os.path.join(OUTPUT_DIR, "out_" + pic)
    cv2.imwrite(output_pic, frame)


def main():
    """
    main
    """
    #create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    #acl init
    acl_resource = AclResource()
    acl_resource.init()

    #load model
    model = Model(MODEL_PATH)
    src_dir = os.listdir(INPUT_DIR)

    #infer picture
    for pic in src_dir:
        if not pic.lower().endswith(('.bmp', '.dib', '.png', '.jpg', 
                                    '.jpeg', '.pbm', '.pgm', '.ppm', '.tif', '.tiff')):
            print('it is not a picture, %s, ignore this file and continue,' % pic)
            continue
        #read picture
        pic_path = os.path.join(INPUT_DIR, pic)
        bgr_img = cv2.imread(pic_path).astype(np.float32)

        #get pic data
        orig_shape, rgb_img = preprocess(bgr_img)

        #inference
        result_list = model.execute([rgb_img, ])    

        #postprocess
        postprocess(result_list, orig_shape, bgr_img, pic)
    print("Execute end")


if __name__ == '__main__':
    main()
