# Copyright (c) OpenMMLab. All rights reserved.

import cv2
from mmdeploy_python import Detector

if __name__ == '__main__':

    img = cv2.imread('demo/resources/det.jpg')
    detector = Detector(
        model_path='mmdeploy_models/mmdet/faster-rcnn/cann', device_name='npu', device_id=0)
    bboxes, labels, masks = detector(img)

    indices = [i for i in range(len(bboxes))]
    for index, bbox, in zip(indices, bboxes):
        [left, top, right, bottom], score = bbox[0:4].astype(int), bbox[4]
        if score < 0.3:
            continue

        cv2.rectangle(img, (left, top), (right, bottom), (0, 255, 0))

        if masks[index].size:
            mask = masks[index]
            blue, green, red = cv2.split(img)
            mask_img = blue[top:top + mask.shape[0], left:left + mask.shape[1]]
            cv2.bitwise_or(mask, mask_img, mask_img)
            img = cv2.merge([blue, green, red])

    cv2.imwrite('output_detection.png', img)