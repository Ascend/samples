"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-11 10:12:13
MODIFIED: 2020-12-11 14:04:45
"""
import sys
import math
import operator
import functools 
from PIL import Image



def image_contrast(image1, image2):
    """
    Verify that the pictures are the same
    """
    file1 = Image.open(image1)
    file2 = Image.open(image2)
    h1 = file1.histogram()
    h2 = file2.histogram()
    ret = math.sqrt(functools.reduce(operator.add, list(map(lambda a, b: (a - b) ** 2, h1, h2))) / len(h1))
    return ret



if __name__ == '__main__':
    img1 = sys.argv[1]
    img2 = sys.argv[2]
    result = image_contrast(img1, img2)
    
    if (result < 0.0):
        sys.exit(1)
    else:
        sys.exit(0)
