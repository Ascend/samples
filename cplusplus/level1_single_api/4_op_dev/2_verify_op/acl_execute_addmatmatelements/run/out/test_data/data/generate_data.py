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

a = np.random.randint(100, size=(3, 32)).astype(np.float32)
b = np.random.randint(100, size=(3, 32)).astype(np.float32)
c = np.random.randint(100, size=(3, 32)).astype(np.float32)
d = np.random.randint(100, size=(1)).astype(np.float32)
e = np.random.randint(100, size=(1)).astype(np.float32)

a.tofile('input_0.bin')
b.tofile('input_1.bin')
c.tofile('input_2.bin')
d.tofile('input_3.bin')
e.tofile('input_4.bin')
