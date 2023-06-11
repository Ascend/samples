"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2023-02-20 10:12:13
"""
import sys
import math
import numpy as np


def data_contrast(file1, file2):
    """
    Verify that the data are the same
    """
    data1 = np.fromfile(file1, dtype=np.int32)
    data2 = np.fromfile(file2, dtype=np.int32)

    if (str(data1) == str(data2)):
        return 0
    else:
        return 1

if __name__ == '__main__':
    input1 = sys.argv[1]
    input2 = sys.argv[2]
    output = sys.argv[3]
    result = data_contrast(input1, output)
    shape = np.fromfile(input2, dtype=np.int32)
    output_shape = '[2 2]'

    if (result == 0) and (str(shape) == str(output_shape)):
        sys.exit(0)
    else:
        sys.exit(1)
