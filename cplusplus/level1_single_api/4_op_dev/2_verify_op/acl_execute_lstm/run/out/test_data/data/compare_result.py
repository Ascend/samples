"""
* @file compare_result.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np


def compare(golden, result):
    return np.sum(abs(golden-result) > 0.01) < 0.01*result.size


if __name__ == '__main__':
    aicore_result_h = np.fromfile("../../result_files/output_0.bin",
                                  dtype="float16")
    aicore_result_c = np.fromfile("../../result_files/output_1.bin",
                                  dtype="float16")

    numpy_result_h = np.fromfile("output_golden_0.bin", dtype="float16")
    numpy_result_c = np.fromfile("output_golden_1.bin", dtype="float16")

    if compare(numpy_result_h, aicore_result_h) and\
            compare(numpy_result_c, aicore_result_c):
        print("compare success")
    else:
        print("compare failed")
