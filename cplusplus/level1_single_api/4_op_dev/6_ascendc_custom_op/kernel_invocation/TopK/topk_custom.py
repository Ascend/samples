#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np


def gen_golden_data():
    input_size = 2097152
    workspace_size = input_size * 8
    x = np.random.uniform(-60000, 60000, [input_size]).astype(np.float16)
    workspace = np.random.uniform(-100, 100, [workspace_size]).astype(np.float16)
    sync = np.zeros([128]).astype(np.int32)
    golden = (np.sort(x)).astype(np.float16)

    param = np.zeros((1, 8), np.int32)
    total_len = input_size
    k = input_size
    is_ascend = 0
    golden = (np.sort(x)).astype(np.float16)
    if is_ascend == 0:
        golden = (-np.sort(-x)).astype(np.float16)
    param[0][0] = k
    param[0][1] = total_len
    param[0][2] = is_ascend

    golden = golden[0:k]
    x.tofile("./input/input_x.bin")
    workspace.tofile("./input/workspace.bin")
    sync.tofile("./input/sync.bin")
    param.tofile("./input/param.bin")
    golden.tofile("./output/golden.bin")


if __name__ == "__main__":
    gen_golden_data()