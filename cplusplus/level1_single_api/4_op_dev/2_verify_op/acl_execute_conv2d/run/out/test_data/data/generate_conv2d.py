"""
* @file generate_conv2d.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np

FEATURE_MAP = np.random.uniform(-5, 5, size=(8, 256, 7, 7)).astype(np.float16)
WEIGHT = np.random.uniform(-5, 5, size=(256, 256, 3, 3)).astype(np.float16)
FEATURE_MAP.tofile('input_0.bin')
WEIGHT.tofile('input_1.bin')