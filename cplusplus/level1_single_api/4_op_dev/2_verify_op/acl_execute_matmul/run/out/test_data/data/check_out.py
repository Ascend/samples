"""
* @file check_out.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np

input_0 = np.fromfile("./input_0.bin", dtype=np.float16).reshape(16, 64)
input_1 = np.fromfile("./input_1.bin", dtype=np.float16).reshape(64, 1024)

TIK_RESULT = np.fromfile("../../result_files/output_0.bin", dtype=np.float16).reshape(16, 1024)
NP_RESULT = np.matmul(input_0, input_1).astype('float16').reshape(16, 1024)

print("The matmulTik result is: \n", TIK_RESULT)
if (NP_RESULT == TIK_RESULT).all():
    print("Compared with the numpy calculation method, the result is correct.")
else:
    print("Compared with the numpy calculation method, the result is wrong.")
