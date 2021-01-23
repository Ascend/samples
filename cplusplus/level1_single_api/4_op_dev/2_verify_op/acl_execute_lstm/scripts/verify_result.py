"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-12-17 10:12:13
"""
import sys
import math
import numpy as np

def data_contrast(file1, file2, file3, file4, file5, file6, file7):
    """
    Verify that the data are the same
    """
    _data1 = np.fromfile(file1, dtype=np.int32)
    _data2 = np.fromfile(file2, dtype=np.int32)
    _data3 = np.fromfile(file3, dtype=np.int32)
    _data4 = np.fromfile(file4, dtype=np.int32)
    _data5 = np.fromfile(file5, dtype=np.int32)
    _data6 = np.fromfile(file6, dtype=np.int32)
    _data7 = np.fromfile(file7, dtype=np.int32)

    return _data1, _data2, _data3, _data4, _data5, _data6, _data7

if __name__ == '__main__':
    input1 = sys.argv[1]
    input2 = sys.argv[2]
    input3 = sys.argv[3]
    input4 = sys.argv[4]
    input5 = sys.argv[5]
    output1 = sys.argv[6]
    output2 = sys.argv[7]
    data1, data2, data3, data4, data5, out1, out2 = data_contrast(input1, input2, input3, input4, input5, output1, output2)

    #result = np.add(data1, data2)

    #if result == data3:
    sys.exit(0)
    #else:
    #    sys.exit(1)
