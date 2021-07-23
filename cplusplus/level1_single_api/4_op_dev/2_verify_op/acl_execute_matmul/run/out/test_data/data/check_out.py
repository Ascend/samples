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

output_data = np.fromfile("../../result_files/output_0.bin", dtype=np.float16)
NP_RESULT = np.fromfile("./np_result.bin", dtype=np.float32).reshape(16, 1024)
out_old = output_data.reshape(16, 1024)
out_new = np.zeros((16, 1024), dtype=np.float16)
print("The matmulTik result is: \n", out_old)
if (NP_RESULT == out_old).all():
    print("Compared with the numpy calculation method, the result is correct.")
else:
    print("Compared with the numpy calculation method, the result is wrong.")
