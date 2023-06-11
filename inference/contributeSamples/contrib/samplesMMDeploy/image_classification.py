# Copyright (c) OpenMMLab. All rights reserved.

import cv2
from mmdeploy_python import Classifier

def main():

    img = cv2.imread('tests/data/tiger.jpeg')
    classifier = Classifier(
        model_path='mmdeploy_models/mmcls/resnet18/cann', device_name='npu', device_id=0)
    result = classifier(img)
    for label_id, score in result:
        print(label_id, score)


if __name__ == '__main__':
    main()
