#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np


def gen_golden_data():
    x1_gm_type = np.float16
    x2_gm_type = np.float16

    M = 32
    N = 32
    K = 32

    x1_gm = np.random.randint(1, 10, [M, K]).astype(x1_gm_type)
    x2_gm = np.random.randint(1, 10, [K, N]).astype(x2_gm_type)
    golden = np.matmul(x1_gm.astype(np.float32), x2_gm.astype(np.float32)).astype(np.float32)

    x1_gm.tofile("./input/x1_gm.bin")
    x2_gm.tofile("./input/x2_gm.bin")
    golden.tofile("./output/golden.bin")


if __name__ == "__main__":
    gen_golden_data()
