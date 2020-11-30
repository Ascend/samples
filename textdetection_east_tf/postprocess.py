import os
import cv2
import numpy as np
import sys
import time
from icdar import restore_rectangle
from preprocess import resize_image
import lanms

def detect(score_map, geo_map, timer, score_map_thresh=0.8, box_thresh=0.1, nms_thres=0.2):
    '''
    restore text boxes from score map and geo map
    :param score_map:
    :param geo_map:
    :param timer:
    :param score_map_thresh: threshhold for score map
    :param box_thresh: threshhold for boxes
    :param nms_thres: threshold for nms
    :return:
    '''
    if len(score_map.shape) == 4:
        score_map = score_map[0, :, :, 0]
        geo_map = geo_map[0, :, :, ]
    # filter the score map
    xy_text = np.argwhere(score_map > score_map_thresh)
    # sort the text boxes via the y axis
    xy_text = xy_text[np.argsort(xy_text[:, 0])]
    # restore
    start = time.time()
    text_box_restored = restore_rectangle(xy_text[:, ::-1]*4, geo_map[xy_text[:, 0], xy_text[:, 1], :]) # N*4*2
    print('{} text boxes before nms'.format(text_box_restored.shape[0]))
    boxes = np.zeros((text_box_restored.shape[0], 9), dtype=np.float32)
    boxes[:, :8] = text_box_restored.reshape((-1, 8))
    boxes[:, 8] = score_map[xy_text[:, 0], xy_text[:, 1]]
    timer['restore'] = time.time() - start
    # nms part
    start = time.time()
    # boxes = nms_locality.nms_locality(boxes.astype(np.float64), nms_thres)
    boxes = lanms.merge_quadrangle_n9(boxes.astype('float32'), nms_thres)
    timer['nms'] = time.time() - start

    if boxes.shape[0] == 0:
        return None, timer

    # here we filter some low score boxes by the average score map, this is different from the orginal paper
    for i, box in enumerate(boxes):
        mask = np.zeros_like(score_map, dtype=np.uint8)
        cv2.fillPoly(mask, box[:8].reshape((-1, 4, 2)).astype(np.int32) // 4, 1)
        boxes[i, 8] = cv2.mean(score_map, mask)[0]
    boxes = boxes[boxes[:, 8] > box_thresh]

    return boxes, timer

def sort_poly(p):
    min_axis = np.argmin(np.sum(p, axis=1))
    p = p[[min_axis, (min_axis+1)%4, (min_axis+2)%4, (min_axis+3)%4]]
    if abs(p[0, 0] - p[1, 0]) > abs(p[0, 1] - p[1, 1]):
        return p
    else:
        return p[[0, 3, 2, 1]]

if __name__ == "__main__":
    src_path = sys.argv[1]
    npu_output_path = sys.argv[2]
    pic_output_path = sys.argv[3]
    # src_path = "D:\code\modelzoo.nkxiaolei\modelzoo\contrib\ATC_EAST_tf_nkxiaolei\image_input"
    # npu_output_path = "D:\code\modelzoo.nkxiaolei\modelzoo\contrib\ATC_EAST_tf_nkxiaolei\output"
    # pic_output_path = "D:\code\modelzoo.nkxiaolei\modelzoo\contrib\ATC_EAST_tf_nkxiaolei\image_output"

    files = os.listdir(src_path)
    files.sort()
    for file in files:
        if file.endswith('.JPEG') or file.endswith('.jpg') or file.endswith('.png'):
            src = src_path + "/" + file
            print("start to process %s" % src)
            im = cv2.imread(src)[:, :, ::-1]
            im_resized, (ratio_h, ratio_w) = resize_image(im)

            score = np.fromfile(npu_output_path + "/" + file+"_output_0.bin",dtype='float32').reshape(1,192,192,1)
            geometry = np.fromfile(npu_output_path + "/" + file+"_output_1.bin",dtype='float32').reshape(1,192,192,5)
            timer = {'net': 0, 'restore': 0, 'nms': 0}
            boxes, timer = detect(score_map=score, geo_map=geometry, timer=timer)

            if boxes is not None:
                boxes = boxes[:, :8].reshape((-1, 4, 2))
                boxes[:, :, 0] /= ratio_w
                boxes[:, :, 1] /= ratio_h

                #save to pictures
                res_file = os.path.join(
                    pic_output_path,
                    '{}.txt'.format(
                        os.path.basename(file).split('.')[0]))

                with open(res_file, 'w') as f:
                    for box in boxes:
                        # to avoid submitting errors
                        box = sort_poly(box.astype(np.int32))
                        if np.linalg.norm(box[0] - box[1]) < 5 or np.linalg.norm(box[3]-box[0]) < 5:
                            continue
                        f.write('{},{},{},{},{},{},{},{}\r\n'.format(
                            box[0, 0], box[0, 1], box[1, 0], box[1, 1], box[2, 0], box[2, 1], box[3, 0], box[3, 1],
                        ))
                        cv2.polylines(im[:, :, ::-1], [box.astype(np.int32).reshape((-1, 1, 2))], True, color=(255, 255, 0), thickness=1)
                img_path = os.path.join(pic_output_path, file)
                cv2.imwrite(img_path, im[:, :, ::-1])
