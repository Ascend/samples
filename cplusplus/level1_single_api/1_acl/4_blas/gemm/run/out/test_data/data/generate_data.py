"""
* @file generate_data.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np

m = 16
k = 16
n = 16
alpha = np.float16(2.0)
beta = np.float16(1.0)

a = np.random.rand(m, k).astype(np.float16)
b = np.random.rand(k, n).astype(np.float16)
c = np.random.rand(m, n).astype(np.float16)

a.tofile('matrix_a.bin')
b.tofile('matrix_b.bin')
c.tofile('matrix_c.bin')

m = np.dot(a.astype(np.float32), b.astype(np.float32))
out = alpha * m + beta * c
out = out.astype(np.float16)

out.tofile('output.bin')

print("Output:")
print(out)

