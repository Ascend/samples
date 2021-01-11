"""
* @file generate_datatik.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np

a = np.random.randint(-5, 5, size=(16, 64)).astype(np.float16)
b = np.random.randint(-5, 5, size=(64, 1024)).astype(np.float16)
res = np.matmul(a, b)
a_z = np.zeros((4, 16, 16), dtype=np.float16)
b_z = np.zeros((4, 1024, 16), dtype=np.float16)
for k in range(4):
    for m in range(16):
        for c in range(16):
            k_idx = k * 16 + c
            if m < 16 and k < 64:
                a_z[k, m, c] = a[m, k_idx]

for k in range(4):
    for n in range(1024):
        for c in range(16):
            k_idx = k * 16 + c
            if k_idx < 64 and n < 1024:
                b_z[k, n, c] = b[k_idx, n]

a_z.reshape(16, 64).tofile('input_0.bin')
b_z.reshape(64, 1024).tofile('input_1.bin')
res.tofile('np_result.bin')
print(res)
