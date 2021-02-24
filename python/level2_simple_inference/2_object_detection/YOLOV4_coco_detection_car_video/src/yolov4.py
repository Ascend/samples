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
import LaneFinder
sys.path.append("../../../../common/atlas_utils")
sys.path.append("../../../../common")
sys.path.append("../")
from acl_model import Model
from acl_resource import AclResource

MODEL_WIDTH = 416
MODEL_HEIGHT = 416
OUTPUT_DIR = '../out/'
MODEL_PATH = '../model/yolov4_no_postprocess.om'
CLASS_NUM = 80
NMS_THRESHOLD_CONST = 0.5
CLASS_SCORE_CONST = 0.4
MODEL_OUTPUT_BOXNUM = 10647
TRAFFIC = [0, 1, 2, 3, 4, 5, 6, 7, 8]
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


def calculate_position(bbox, transform_matrix, warped_size, pix_per_meter, x_scale, y_scale):
    """
    calculate_position
    """
    if len(bbox) == 0:
        print('Nothing')
    else:
        top_x= int((bbox[0] - bbox[2] / 2) * x_scale)
        top_y= int((bbox[1] - bbox[3] / 2) * y_scale)
        bottom_x= int((bbox[0] + bbox[2] / 2) * x_scale)
        bottom_y= int((bbox[1] + bbox[3] / 2) * y_scale)
        point = np.array((top_x / 2 + bottom_x / 2, bottom_y)).reshape(1, 1, -1)
        pos = cv2.perspectiveTransform(point, transform_matrix).reshape(-1, 1)
        return np.array((warped_size[1] - pos[1]) / pix_per_meter[1])


def preprocess_frame(frame):
    """
    preprocess_frame
    """
    frame = frame[:, :, ::-1]
    image = frame
    image = LaneFinder.Image.fromarray(image.astype('uint8'), 'RGB')
    
    return image


def preprocess(frame):
    """
    preprocess
    """
    #get img shape
    orig_shape = frame.shape[:2]

    rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #normalization
    rgb_img = rgb_img / 255.0
    #resize img
    rgb_img = cv2.resize(rgb_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
    # save memory C_CONTIGUOUS mode
    if not rgb_img.flags['C_CONTIGUOUS']:
        rgb_img = np.ascontiguousarray(frame)
    
    frame = preprocess_frame(frame)
    fframe = np.array(frame)
    fframe = lf.process_image(fframe, False)
    frame = LaneFinder.Image.fromarray(fframe)
    framecv = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)

    return orig_shape, rgb_img, framecv


def postprocess(result_list, frame, orig_shape):
    """
    postprocess
    """
    x_scale = orig_shape[1] / MODEL_HEIGHT
    y_scale = orig_shape[0] / MODEL_WIDTH

    result_class = result_list[0].reshape(MODEL_OUTPUT_BOXNUM, 80).astype('float32')
    result_box = result_list[1].reshape(MODEL_OUTPUT_BOXNUM, 4).astype('float32')
    boxes = np.zeros(shape = (MODEL_OUTPUT_BOXNUM, 6), dtype = np.float32)
    boxes[:, :4]= result_box
    list_score = result_class.max(axis = 1)
    list_class = result_class.argmax(axis=1)
    list_score = list_score.reshape(MODEL_OUTPUT_BOXNUM, 1)
    list_class=list_class.reshape(MODEL_OUTPUT_BOXNUM, 1)
    boxes[:, 4]= list_class[:, 0]
    boxes[:, 5]= list_score[:, 0]
    all_boxes = boxes[boxes[:, 5] >= CLASS_SCORE_CONST]
    real_box = func_nms(np.array(all_boxes), NMS_THRESHOLD_CONST)

    l = len(real_box)
    distance = np.zeros(shape=(l, 1))
    if not len(real_box) == 0:
        for i in range(l):
            distance[i] = calculate_position(bbox=real_box[i],
                                             transform_matrix=perspective_transform,
                                             warped_size=WARPED_SIZE,
                                             pix_per_meter=pixels_per_meter, 
                                             x_scale=x_scale, y_scale=y_scale)
        print('RPOS', distance)
    else:
        distance = []
        print('No Car')

    for i, detect_result in enumerate(real_box):
        if int(detect_result[4]) in TRAFFIC:
            top_x= int((detect_result[0] - detect_result[2] / 2) * x_scale)
            top_y= int((detect_result[1] - detect_result[3] / 2) * y_scale)
            bottom_x= int((detect_result[0] + detect_result[2] / 2) * x_scale)
            bottom_y= int((detect_result[1] + detect_result[3] / 2) * y_scale)
            cv2.rectangle(frame, (top_x, top_y), (bottom_x, bottom_y), (0, 255, 0), 2)
            cv2.putText(frame, labels[int(detect_result[4])], (top_x + 5, top_y + 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            d = distance[i]
            label_dis = '{} {:.2f}m'.format('dis:', d[0])
            cv2.putText(frame, label_dis, (top_x + 10, bottom_y + 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    return frame


def main():
    """
    main
    """
    if (len(sys.argv) != 2):
        print("Please input video path")
        exit(1)

    #acl init
    acl_resource = AclResource()
    acl_resource.init()
    #load model
    model = Model(MODEL_PATH)

    #open video
    video_path = sys.argv[1]
    print("open video ", video_path)
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    Width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    Height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    lf.set_img_size((Width, Height))

    #create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    output_Video = os.path.basename(video_path)
    output_Video = os.path.join(OUTPUT_DIR, output_Video)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # DIVX, XVID, MJPG, X264, WMV1, WMV2
    outVideo = cv2.VideoWriter(output_Video, fourcc, fps, (Width, Height))

    # Read until video is completed
    while (cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            #preprocess
            orig_shape, rgb_img, framecv = preprocess(frame)
            #inference
            result_list = model.execute([rgb_img, ]) 
            #postprocess
            frame = postprocess(result_list, framecv, orig_shape)
            outVideo.write(frame)
        # Break the loop
        else:
            break
    cap.release()
    outVideo.release()
    print("Execute end")


if __name__ == '__main__':

    import gc
    path = './configure.json'
    config_file = open(path, "rb")
    fileJson = json.load(config_file)
    cam_matrix = fileJson[0]["cam_matrix"]
    dist_coeffs = fileJson[0]["dist_coeffs"]
    perspective_transform = fileJson[0]["perspective_transform"]
    pixels_per_meter = fileJson[0]["pixels_per_meter"]
    WARPED_SIZE = fileJson[0]["WARPED_SIZE"]
    ORIGINAL_SIZE = fileJson[0]["ORIGINAL_SIZE"]

    cam_matrix = np.array(cam_matrix)
    dist_coeffs = np.array(dist_coeffs)
    perspective_transform = np.array(perspective_transform)
    pixels_per_meter = tuple(pixels_per_meter)
    WARPED_SIZE = tuple(WARPED_SIZE)
    ORIGINAL_SIZE = tuple(ORIGINAL_SIZE)

    lf = LaneFinder.LaneFinder(ORIGINAL_SIZE, WARPED_SIZE, cam_matrix, dist_coeffs,
                    perspective_transform, pixels_per_meter)
    main()
    gc.collect()
