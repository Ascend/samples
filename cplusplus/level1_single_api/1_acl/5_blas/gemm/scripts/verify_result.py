"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-11 10:12:13
MODIFIED: 2020-12-11 14:04:45
"""
import sys
import filecmp


def image_contrast(image1, image2):
    """
    Verify that the pictures are the same
    """
   
    ret = filecmp.cmp(image1,image2, shallow=True)
    return ret



if __name__ == '__main__':
    img1_file = sys.argv[1]
    img2_file = sys.argv[2]
    result = image_contrast(img1_file, img2_file)
    if result:
        sys.exit(0)
    else:
        sys.exit(1)
