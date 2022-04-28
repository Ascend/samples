import cv2 as cv
import numpy as np
import os
import sys
import time

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(path, ".."))
sys.path.append(os.path.join(path, "../../../../common/"))
sys.path.append(os.path.join(path, "../../../../common/acllite"))

from constants import IMG_EXT
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage
from acllite_resource import AclLiteResource

from collections import defaultdict

from config import config
import json

MODEL_WIDTH = 320
MODEL_HEIGHT = 320

INPUT_DIR = '../data/'
OUTPUT_DIR = '../out/'
model_path = '../model/ssd_500.om'

means = [0.485, 0.456, 0.406]
stds = [0.229, 0.224, 0.225]

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

def preprocess(bgr_img):
    
    # bgr_img = cv.imread(picPath).astype(np.float32)
    print("original pic shape:", bgr_img.shape)

    # original_h, original_w = bgr_img.shape[0:2]
    
    bgr_img = cv.resize(bgr_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)   
 
    rgb_img = cv.cvtColor(bgr_img, cv.COLOR_BGR2RGB)
    print("rgb_img:", rgb_img.shape)    
    
    # covert from NHWC to NCHW
    rgb_img = rgb_img.transpose(2, 0, 1).copy()
    rgb_img = rgb_img / 255.0
    
    rgb_img[0] = (rgb_img[0] - means[0])/stds[0]
    rgb_img[1] = (rgb_img[1] - means[1])/stds[1]
    rgb_img[2] = (rgb_img[2] - means[2])/stds[2]
    
    if not rgb_img.flags['C_CONTIGUOUS']:
        rgb_img = np.ascontiguousarray(rgb_img)

    # l_data = l_data - 50
    return rgb_img

def postprocess(result_list, pic, bgr_img):
    boxes = result_list[0][0]
    box_scores = result_list[1][0]

    # original_h, original_w = bgr_img.shape[0:2]
    
    # print(len(results))
    pred_data = []

    pred_data.append({
            "boxes": boxes,
            "box_scores": box_scores,
            "img_id": pic,
            "image_shape": bgr_img.shape[0:2]
        })

    predictions = metrics(pred_data)

    drawLable(predictions, bgr_img, pic)



    
def apply_nms(all_boxes, all_scores, thres, max_boxes):
    """Apply NMS to bboxes."""
    y1 = all_boxes[:, 0]
    x1 = all_boxes[:, 1]
    y2 = all_boxes[:, 2]
    x2 = all_boxes[:, 3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)

    order = all_scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        if len(keep) >= max_boxes:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h

        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= thres)[0]

        order = order[inds + 1]
    return keep


def metrics(pred_data):
    """Calculate mAP of predicted bboxes."""
    num_classes = config.num_classes

    #Classes need to train or test.
    val_cls = config.classes
    val_cls_dict = {}
    for i, cls in enumerate(val_cls):
        val_cls_dict[i] = cls
    # coco_gt = COCO(anno_json)
    # classs_dict = {}
    # cat_ids = coco_gt.loadCats(coco_gt.getCatIds())
    # for cat in cat_ids:
    #     classs_dict[cat["name"]] = cat["id"]

    predictions = []
    img_ids = []

    for sample in pred_data:
        pred_boxes = sample['boxes']
        box_scores = sample['box_scores']
        img_id = sample['img_id']
        h, w = sample['image_shape']

        final_boxes = []
        final_label = []
        final_score = []
        img_ids.append(img_id)

        for c in range(1, num_classes):
            class_box_scores = box_scores[:, c]
            score_mask = class_box_scores > config.min_score
            class_box_scores = class_box_scores[score_mask]
            class_boxes = pred_boxes[score_mask] * [h, w, h, w]

            if score_mask.any():
                nms_index = apply_nms(class_boxes, class_box_scores, config.nms_threshold, config.max_boxes)
                class_boxes = class_boxes[nms_index]
                class_box_scores = class_box_scores[nms_index]

                final_boxes += class_boxes.tolist()
                final_score += class_box_scores.tolist()
                # final_label += [classs_dict[val_cls_dict[c]]] * len(class_box_scores)
                final_label += [val_cls_dict[c]] * len(class_box_scores)

        for loc, label, score in zip(final_boxes, final_label, final_score):
            res = {}
            res['image_id'] = img_id
            # res['bbox'] = [loc[1], loc[0], loc[3] - loc[1], loc[2] - loc[0]]
            res['bbox'] = loc
            res['score'] = score
            res['category_id'] = label
            predictions.append(res)
    
    print(predictions)
    with open('predictions.json', 'w') as f:
        json.dump(predictions, f)

    return predictions
    
def drawLable(results, bgr_img, pic):
    i = 0
    for res in results:
        box = res['bbox']
        class_name = res['category_id']
        confidence = res['score']
        cv.rectangle(bgr_img, (int(box[1]), int(box[0])), (int(box[3]), int(box[2])), colors[i%6])
        p3 = (max(int(box[1]), 15), max(int(box[0]), 15))
        out_label = class_name            
        cv.putText(bgr_img, out_label, p3, cv.FONT_ITALIC, 0.6, colors[i % 6], 1)

        i += 1

    output_file = os.path.join(OUTPUT_DIR, "output" + os.path.basename(pic))
    print("output:%s" % output_file)
    cv.imwrite(output_file, bgr_img)

def main():
    """
    acl resource initialization
    """
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    #ACL resource initialization    
    acl_resource = AclLiteResource()
    acl_resource.init()
    
    model = AclLiteModel(model_path)
    images_list = [os.path.join(INPUT_DIR, img)
                   for img in os.listdir(INPUT_DIR)
                   if os.path.splitext(img)[1] in IMG_EXT]

    for pic in images_list:
        print("pic: ", pic)

        bgr_img = cv.imread(pic).astype(np.float32)               
        data = preprocess(bgr_img)

        result_list = model.execute([data])

        postprocess(result_list, pic, bgr_img)

    print("Execute end")

if __name__ == '__main__':
    main()
