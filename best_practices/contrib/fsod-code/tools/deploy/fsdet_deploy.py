import os
import sys
import argparse
from PIL import Image

import acl
import cv2 as cv
import numpy as np
from tqdm import tqdm

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource

from metric import PascalVOCDetectionEvaluator

# Custom dataset's labels
dataset_name = "Jiangxiang-VOC-test"
labels = ["hole", "crack"]
colors = [(255, 0, 0), (0, 255, 0)]

mode = "metric"
split = "test"
DATA_DIR = "datasets/VOC-custom/"
TEST_TXT = "datasets/VOC-custom/ImageSets/Main/test.txt"
INPUT_DIR = "datasets/VOC-custom/JPEGImages/"
OUTPUT_DIR = "output"
MODEL_PATH = "fsdet_model.om"
MODEL_WIDTH = 800
MODEL_HEIGHT = 800

iou_threshold = 0.
confidence_threshold = 0.

class_num = len(labels)

def preprocess(img_path):
    image = Image.open(img_path)
    img_h = image.size[1]
    img_w = image.size[0]
    net_h = MODEL_HEIGHT
    net_w = MODEL_WIDTH

    scale = min(float(net_w) / float(img_w), float(net_h) / float(img_h))
    new_w = int(img_w * scale)
    new_h = int(img_h * scale)

    shift_x = (net_w - new_w) // 2
    shift_y = (net_h - new_h) // 2
    shift_x_ratio = (net_w - new_w) / 2.0 / net_w
    shift_y_ratio = (net_h - new_h) / 2.0 / net_h

    image_ = image.resize((new_w, new_h))
    new_image = np.zeros((net_h, net_w, 3), np.uint8)
    new_image[shift_y: new_h + shift_y, shift_x: new_w + shift_x, :] = np.array(image_)
    new_image = new_image.astype(np.float32)
    new_image = new_image.transpose(2, 0, 1).copy()
    return new_image, image

def overlap(x1, x2, x3, x4):
    left = max(x1, x3)
    right = min(x2, x4)
    return right - left

def cal_iou(box, truth):
    w = overlap(box[0], box[2], truth[0], truth[2])
    h = overlap(box[1], box[3], truth[1], truth[3])
    if w <= 0 or h <= 0:
        return 0
    inter_area = w * h
    union_area = (box[2] - box[0]) * (box[3] - box[1]) + (truth[2] - truth[0]) * (truth[3] - truth[1]) - inter_area
    return inter_area * 1.0 / union_area

def apply_nms(boxes, scores, thres):
    idx = scores.argsort()
    keep = []
    
    while len(idx) > 0:
        max_score_idx = idx[-1]
        max_score_box = boxes[max_score_idx]
        keep.append(max_score_idx)
        if len(idx) == 1:
            break
        idx = idx[:-1]

        below_thres_idx = []
        other_boxes = boxes[idx]
        for i, box in enumerate(other_boxes):
            if cal_iou(max_score_box, box) <= thres:
                below_thres_idx.append(i)
        idx = idx[below_thres_idx]
    return keep

def post_process(infer_output, origin_img):
    #print("post process")
    result_return = dict()
    img_h = origin_img.size[1]
    img_w = origin_img.size[0]
    scale = min(float(MODEL_WIDTH) / float(img_w), float(MODEL_HEIGHT) / float(img_h))
    new_w = int(img_w * scale)
    new_h = int(img_h * scale)
    shift_x = (MODEL_WIDTH - new_w) / 2
    shift_y = (MODEL_HEIGHT - new_h) / 2

    infer_output[1][:, [0, 2]] = (infer_output[1][:, [0, 2]] - shift_x) / scale
    infer_output[1][:, [1, 3]] = (infer_output[1][:, [1, 3]] - shift_y) / scale

    class_number = len(labels)

    scores = infer_output[0]
    boxes = infer_output[1]
    classes = infer_output[2]
    
    keep = scores > confidence_threshold
    scores = scores[keep]
    boxes = boxes[keep]
    classes = classes[keep]

    keep = apply_nms(boxes, scores, iou_threshold)

    if not keep:
        result_return['detection_classes'] = []
        result_return['detection_boxes'] = []
        result_return['detection_scores'] = []
        return result_return
    else:
        result_return['detection_classes'] = classes[keep]
        result_return['detection_boxes'] = boxes[keep]
        result_return['detection_scores'] = scores[keep]
        return result_return


def main():
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    
    # ACL resource initialization
    acl_resource = AclLiteResource()
    acl_resource.init()
    
    # load model
    model = AclLiteModel(MODEL_PATH)

    # Load image path
    images_list = [os.path.join(INPUT_DIR, img_id[:-1] + ".jpg")
            for img_id in open(TEST_TXT, "r")]
                   #if os.path.splitext(img)[1] in const.IMG_EXT]
    
    print("IOU Threshold: ", iou_threshold)
    print("CF Threshold: ", confidence_threshold)

    import time
    time_list = []
    result_dict = {}
    img_id_list = []
    # Read images from the data directory one by one for reasoning
    for pic in tqdm(images_list, desc="Inferencing"):
        img_id = os.path.basename(pic).split(".")[0]

        img_id_list.append(img_id)

        # read image
        bgr_img = cv.imread(pic)
        
        # preprocess
        data, orig = preprocess(pic)
        
        # Send into model inference
        start_time = time.time()
        result_list = model.execute([data,])
        time_list.append(time.time() - start_time)

        # Process predict results
        result_return = post_process(result_list, orig)
        result_dict[img_id] = result_return

        if mode == "metric":
            continue
        
        datacv = bgr_img.copy()
        for i in range(len(result_return["detection_boxes"])):
            label_id = result_return["detection_classes"][i]
            label = labels[label_id]
            box = result_return["detection_boxes"][i]
            score = result_return["detection_scores"][i]
            cv.rectangle(datacv, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), colors[label_id % 2], thickness=1)
            p3 = (max(int(box[0]), 15), max(int(box[1]), 15))
            cv.putText(datacv, label, p3, cv.FONT_HERSHEY_COMPLEX, 0.6, colors[label_id % 2], 1)
        
        output_file = os.path.join(OUTPUT_DIR, "out_" + os.path.basename(pic))
        cv.imwrite(output_file, datacv)
    print("Average Inference Time: {} s/img".format(sum(time_list) / len(time_list)))

    evaluator = PascalVOCDetectionEvaluator(dataset_name, labels, DATA_DIR, split)
    evaluator.reset()
    evaluator.process(img_id_list, result_dict)
    evaluator.evaluate()

    print("Execute end")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run inference of Fsdet om model')
    parser.add_argument("--model", type=str, default="./fsdet_model.om", help="om model path")
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--mode", type=str, default="metric", choices=["metric", "predict"])
    parser.add_argument("--txt", type=str, default="datasets/VOC-custom/ImageSets/Main/test.txt", help="the image id list txt file")
    parser.add_argument("--split", type=str, default="test")
    parser.add_argument("--data_dir", type=str, default="datasets/VOC-custom/")
    parser.add_argument("--cf_thres", type=float, default=0.05)
    parser.add_argument("--iou_thres", type=float, default=0.5)
    args = parser.parse_args()

    mode = args.mode
    split = args.split
    DATA_DIR = args.data_dir
    TEST_TXT = args.txt
    INPUT_DIR = os.path.join(DATA_DIR, "JPEGImages")
    OUTPUT_DIR = "output"
    MODEL_PATH = args.model
    MODEL_HEIGHT = args.height
    MODEL_WIDTH = args.width

    
    iou_threshold = args.iou_thres
    confidence_threshold = args.cf_thres

    import gc

    main()
    gc.collect()
