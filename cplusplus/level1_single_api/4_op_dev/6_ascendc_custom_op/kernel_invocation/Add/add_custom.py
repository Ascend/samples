#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import numpy as np


def gen_golden_data(params, data_dir):
    input_x = np.random.uniform(-100, 100, params[0].shape).astype(params[0].np_dtype)
    input_y = np.random.uniform(-100, 100, params[1].shape).astype(params[1].np_dtype)
    golden = (input_x + input_y).astype(params[2].np_dtype)

    input_x.tofile(str(data_dir / params[0].data_path))
    input_y.tofile(str(data_dir / params[1].data_path))
    golden.tofile(str(data_dir / params[2].golden_path))


def gen_golden_data_simple():
    input_x = np.random.uniform(-100, 100, [8, 2048]).astype(np.float16)
    input_y = np.random.uniform(-100, 100, [8, 2048]).astype(np.float16)
    golden = (input_x + input_y).astype(np.float16)

    input_x.tofile("./input/input_x.bin")
    input_y.tofile("./input/input_y.bin")
    golden.tofile("./output/golden.bin")


if __name__ == "__main__":
    gen_golden_data_simple()
