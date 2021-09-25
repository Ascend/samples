"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-17 10:12:13
"""
import sys
import math
import numpy as np

def data_contrast(file1, file2, file3):
    """
    Verify that the data are the same
    """
    _data1 = np.fromfile(file1, dtype=np.int32)
    _data2 = np.fromfile(file2, dtype=np.int32)
    _data3 = np.fromfile(file3, dtype=np.int32)

    return _data1, _data2, _data3

if __name__ == '__main__':
    input1 = sys.argv[1]
    input2 = sys.argv[2]
    output = sys.argv[3]
    data1, data2, data3 = data_contrast(input1, input2, output)
    result = np.add(data1, data2)

    if all(result == data3):
        sys.exit(0)
    else:
        sys.exit(1)
