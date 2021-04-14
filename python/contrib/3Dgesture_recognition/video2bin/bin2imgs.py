""" bin to images test"""

import cv2
import numpy as np
import os

BINPATH = './Test/test.bin'


BATCH_SIZE = 1
CROP_SIZE = 112
CHANNEL_NUM = 3
CLIP_LENGTH = 16


imgArray = np.fromfile(BINPATH, dtype=np.float32)

imgArray.shape=1, CLIP_LENGTH, CROP_SIZE, CROP_SIZE, CHANNEL_NUM


for i in range(CLIP_LENGTH):
    img = imgArray[0][i]
    #print(img.shape)
    cv2.imwrite('./OutputImages/test' + str(i) + '.jpg', img.astype(int))


