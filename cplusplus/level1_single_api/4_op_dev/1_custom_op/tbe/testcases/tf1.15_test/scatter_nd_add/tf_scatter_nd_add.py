"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this file
except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

"""

from __future__ import absolute_import

from __future__ import division
from __future__ import print_function

# Imports
import logging
import tensorflow as tf
import numpy as np
from npu_bridge.estimator import npu_ops

tf.flags.DEFINE_string("local_log_dir", "output/train_logs.txt", "Log file path")
FLAGS = tf.flags.FLAGS

atol = 0.001
rtol = 0.001

def config(excute_type):
    if excute_type == 'ai_core':
        session_config = tf.ConfigProto(
            allow_soft_placement=True,
            log_device_placement=False)
        custom_op = session_config.graph_options.rewrite_options.custom_optimizers.add()
        custom_op.name = "NpuOptimizer"
        custom_op.parameter_map["enable_data_pre_proc"].b = True
        custom_op.parameter_map["mix_compile_mode"].b = True
        custom_op.parameter_map["use_off_line"].b = True
        custom_op.parameter_map["min_group_size"].b = 1

    elif excute_type == 'cpu':
        session_config = tf.ConfigProto(
            allow_soft_placement=True,
            log_device_placement=False)

    return session_config

def main(unused_argv):
    ref = tf.Variable([[[1, 2, 3, 4], [5, 6, 7, 8],
                        [1, 2, 3, 4], [5, 6, 7, 8]],
                       [[10, 11, 12, 13], [14, 15, 16, 17],
                        [10, 11, 12, 13], [14, 15, 16, 17]]
                       ])
    indices = tf.constant([[1, 1]])
    updates = tf.constant([[-14, -15, -16, -17]])
    out = tf.scatter_nd_add(ref, indices, updates)
    init = tf.global_variables_initializer()

    with tf.Session(config=config('cpu')) as session:
        session.run(init)
        result_cpu = session.run(out)
    with tf.Session(config=config('ai_core')) as session:
        session.run(init)
        result_ai_core = session.run(out)

    print('====================================')
    cmp_result = np.allclose(result_ai_core, result_cpu, atol, rtol)
    print(cmp_result)
    print('====================================')


if __name__ == "__main__":
    tf.app.run()
