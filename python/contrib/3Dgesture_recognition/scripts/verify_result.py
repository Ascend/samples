"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-11 10:12:13
MODIFIED: 2020-12-11 14:04:45
"""
import sys
import numpy as np

def txt_contrast(txtname1, txtname2):
    """
    Verify that the txt are the same
    """
    with open(txtname1, "r", encoding="utf-8") as fp:
        file1 = fp.read()        
    with open(txtname2, "r", encoding="utf-8") as fp:
        file2 = fp.read()

    if(file1 == file2):
        return 1
    else:
        return 0

if __name__ == '__main__':
    txt1 = sys.argv[1]
    txt2 = sys.argv[2]
    result = txt_contrast(txt1, txt2)
    if (result != 1):
        sys.exit(1)
    else:
        sys.exit(0)
