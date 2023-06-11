#!/usr/bin/python3
# -*- coding:utf-8 -*-
# Copyright 2022-2023 Huawei Technologies Co., Ltd
import os
import stat
import numpy as np
OPEN_FILE_MODES_640 = stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP
WRITE_FILE_FLAGS = os.O_WRONLY | os.O_CREAT | os.O_TRUNC


def gen_golden_data_simple():
    one_repeat_calcount = 128  # fixed
    block_dim_imm = 8
    tile_num_imm = 8
    double_buffer_imm = 2  # fixed
    total_length_imm = block_dim_imm * \
        one_repeat_calcount * tile_num_imm * double_buffer_imm

    block_dim = np.array(block_dim_imm, dtype=np.uint32)
    total_length = np.array(total_length_imm, dtype=np.uint32)
    tile_num = np.array(tile_num_imm, dtype=np.uint32)
    tiling = (block_dim, total_length, tile_num)
    tiling_data = b''.join(x.tobytes() for x in tiling)
    with os.fdopen(os.open('./input/tiling.bin', WRITE_FILE_FLAGS, OPEN_FILE_MODES_640), 'wb') as f:
        f.write(tiling_data)

    input_x = np.random.uniform(-100, 100, [total_length_imm]).astype(np.float16)
    input_y = np.random.uniform(-100, 100, [total_length_imm]).astype(np.float16)
    golden = (input_x + input_y).astype(np.float16)

    input_x.tofile("./input/input_x.bin")
    input_y.tofile("./input/input_y.bin")
    golden.tofile("./output/golden.bin")


if __name__ == "__main__":
    gen_golden_data_simple()
