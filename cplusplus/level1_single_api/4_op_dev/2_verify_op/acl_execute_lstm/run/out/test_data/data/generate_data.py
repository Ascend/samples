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


def sigmoid(x):
    s = 1 / (1 + np.exp(-x))
    return s


if __name__ == '__main__':
    hidden_size = 32
    batch_size = 32
    max_time = 16
    feature_size = 32
    forget_bias = 1
    coeff = 1
    b = 0.5
    compute_dtype = "float32"
    np_input_data = (np.random.rand(
        batch_size, max_time, feature_size).astype(compute_dtype) - b) * coeff
    np_rand_bias = (np.random.rand(
        4 * hidden_size) - b).astype(compute_dtype) * coeff

    np_outputs = np.zeros(
        (batch_size, max_time, hidden_size), dtype=compute_dtype)

    np_weight = (np.random.rand(
        feature_size + hidden_size,
        4 * hidden_size) - b).astype(compute_dtype) * coeff

    np_h = np.zeros((batch_size, hidden_size), dtype=compute_dtype)
    np_c = np.zeros((batch_size, hidden_size), dtype=compute_dtype)

    i_w = np_weight[:, 0 * hidden_size:1 * hidden_size]
    j_w = np_weight[:, 1 * hidden_size:2 * hidden_size]
    f_w = np_weight[:, 2 * hidden_size:3 * hidden_size]
    o_w = np_weight[:, 3 * hidden_size:4 * hidden_size]

    for t in range(max_time):
        np_input = np_input_data[:, t, :]
        np_xh = np.concatenate((np_input, np_h), axis=1)
        np_xh = np.expand_dims(np_xh, axis=1)
        i = np.squeeze(sigmoid(np.matmul(np_xh, i_w) +
                               np_rand_bias[0 * hidden_size:1 * hidden_size]))
        j = np.squeeze(np.tanh(np.matmul(np_xh, j_w) +
                               np_rand_bias[1 * hidden_size:2 * hidden_size]))
        f = np.squeeze(sigmoid(np.matmul(np_xh, f_w) +
                               np_rand_bias[2 * hidden_size:3 * hidden_size]
                               + forget_bias))
        o = np.squeeze(sigmoid(np.matmul(np_xh, o_w) +
                               np_rand_bias[3 * hidden_size:4 * hidden_size]))

        np_c = f * np_c + i * j
        np_h = o * np.tanh(np_c)
        np_outputs[:, t, :] = np_h[...]

    block_size = 16
    hidden_blocks = hidden_size // block_size
    feature_blocks = feature_size // block_size
    batch_blocks = batch_size // block_size

    np_input_z = np.zeros(
        (max_time, batch_blocks, feature_blocks, block_size, block_size),
        dtype="float16")

    for t in range(max_time):
        for bid in range(batch_blocks):
            for fid in range(feature_blocks):
                for b0i in range(block_size):
                    for f0i in range(block_size):
                        np_input_z[t, bid, fid, b0i, f0i] =\
                            np_input_data[bid*block_size +
                                          b0i, t, fid*block_size + f0i]

    np_input_z.tofile("input_0.bin")

    np_init_h_z = np.zeros((batch_blocks,
                            hidden_blocks,
                            block_size,
                            block_size), dtype="float16")

    np_init_h_z.tofile("input_1.bin")

    np_init_c_z = np.zeros((batch_blocks,
                            hidden_blocks,
                            block_size,
                            block_size), dtype="float16")

    np_init_c_z.tofile("input_2.bin")

    np_weight_z = np.zeros((feature_blocks + hidden_blocks,
                            4*hidden_blocks,
                            block_size,
                            block_size), dtype="float16")

    for m1 in range(feature_blocks + hidden_blocks):
        for n1 in range(4 * hidden_blocks):
            for m0 in range(block_size):
                for n0 in range(block_size):
                    np_weight_z[m1, n1, n0, m0] =\
                        np_weight[m1 * block_size + m0, n1 * block_size + n0]

    np_weight_z.tofile("input_3.bin")

    np_b_z = np.zeros((4 * hidden_size), dtype="float32")

    for h1 in range(4*hidden_size):
        np_b_z[h1] = np_rand_bias[h1]

    np_b_z.tofile("input_4.bin")

    np_ret_h_z = np.zeros(
        (max_time, batch_blocks, hidden_blocks, block_size, block_size),
        dtype="float16")

    for t in range(max_time):
        for bid in range(batch_blocks):
            for fid in range(hidden_blocks):
                for b0i in range(block_size):
                    for f0i in range(block_size):
                        np_ret_h_z[t, bid, fid, b0i, f0i] =\
                            np_outputs[bid*block_size + b0i,
                                       t, fid*block_size + f0i]

    np_ret_h_z.tofile("output_golden_0.bin")

    np_ret_c_z = np.zeros((batch_blocks,
                           hidden_blocks,
                           block_size,
                           block_size), dtype="float16")

    for bid in range(batch_blocks):
        for hid in range(hidden_blocks):
            for b0i in range(block_size):
                for h0i in range(block_size):
                    np_ret_c_z[bid, hid, b0i, h0i] =\
                        np_c[bid*block_size + b0i, hid * block_size + h0i]

    np_ret_c_z.tofile("output_golden_1.bin")
