# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import tensorflow as tf
from tensorflow.python.framework.graph_util import convert_variables_to_constants
import os
import numpy as np
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
import sys

model_root_path = './'


def create_model(argv):
    test_name = "tf_test"  # 用例名称
    n = 3
    input1 = tf.placeholder(tf.float32, shape=(n, n, n, n), name='input1')
    input2 = tf.placeholder(tf.float32, shape=(n, n, n, n), name='input2')

    a = np.array([[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]])
    const1 = tf.constant(a, dtype=tf.float32, shape=(n, n, n, n), name='const1')

    input1_abs = tf.abs(input1, name='input1_abs')
    input2_abs = tf.abs(input2, name='input2_abs')
    input3_abs = tf.add(input1_abs, const1, name='input3_add')
    output = tf.add(input3_abs, input2_abs, name='output')

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        graph = convert_variables_to_constants(sess, sess.graph_def, ["output"])

        tf.train.write_graph(graph, '.', model_root_path + test_name + '.pb', as_text=False)
        print('Create Model Successful.')
        print('Path: ', model_root_path + test_name + '.pb')

    tf.reset_default_graph()



create_model(sys.argv)
