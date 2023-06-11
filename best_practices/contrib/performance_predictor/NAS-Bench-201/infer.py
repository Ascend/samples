# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# pylint: disable=missing-docstring
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import os
import tensorflow as tf
from npu_bridge.npu_init import *
import operations
from config import FLAGS
from cell import InferCell

from npu_bridge.estimator.npu.npu_loss_scale_manager import ExponentialUpdateLossScaleManager
from npu_bridge.estimator.npu.npu_loss_scale_optimizer import NPULossScaleOptimizer

# Global constants describing the CIFAR-10 data set.
NUM_CLASSES = 10
NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN = 50000
NUM_EXAMPLES_PER_EPOCH_FOR_EVAL = 10000
NUM_EPOCHS_PER_DECAY = 200
TAG = 'NAS_Bench_201:' + os.path.basename(__file__)


def loss_fun(logits, labels):
    cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=tf.one_hot(labels, NUM_CLASSES))

    cross_entropy_mean = tf.reduce_mean(cross_entropy, name='cross_entropy')
    tf.add_to_collection('losses', cross_entropy_mean)

    return tf.add_n(tf.get_collection('losses'), name='total_loss')


def train(total_loss, global_step):
    num_batches_per_epoch = NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN / FLAGS.batch_size
    decay_steps = int(num_batches_per_epoch * NUM_EPOCHS_PER_DECAY)

    lr = tf.train.cosine_decay(FLAGS.initial_lr, global_step, decay_steps, FLAGS.end_lr)

    optimizer = tf.train.MomentumOptimizer(learning_rate=lr, momentum=FLAGS.momentum, use_nesterov=True)

    # loss_scale_manager = ExponentialUpdateLossScaleManager(init_loss_scale=2 ** 32, incr_every_n_steps=1000, decr_every_n_nan_or_inf=2, decr_ratio=0.5)
    # optimizer = NPULossScaleOptimizer(optimizer, loss_scale_manager)

    train_op = optimizer.minimize(total_loss)

    return train_op


def inference(images, genotype, C, N):
    net = tf.layers.conv2d(
                    inputs=images,
                    filters=C,
                    kernel_size=3,
                    padding='same',
                    use_bias=False,
                    kernel_initializer=tf.variance_scaling_initializer()
                )
    net = tf.layers.batch_normalization(inputs=net, axis=3, training=True)

    layer_channels = [C] * N + [C * 2] + [C * 2] * N + [C * 4] + [C * 4] * N
    layer_reductions = [False] * N + [True] + [False] * N + [True] + [False] * N

    C_prev = C

    for index, (C_curr, reduction) in enumerate(zip(layer_channels, layer_reductions)):
        if reduction:
            print('--------------------------------------------------------\n')
            print(TAG, 'ResNet block', index, C_curr, reduction)
            op = operations.ResNetBasicblock(is_training=True, data_format='channels_last')
            net = op.build(inputs=net, channels=C_curr, stride=2)
        else:
            print(TAG, 'Infercell block', index, C_curr, reduction)
            op = InferCell(inputs=net, genotype=genotype, C_out=C_curr, stride=1)
            net = op.build()

    with tf.variable_scope('last_act'):
        net = tf.layers.batch_normalization(inputs=net, axis=3, training=True)
        net = tf.nn.relu(net)
    print(TAG, net)

    with tf.variable_scope('AdaptiveAvgPool2D'):
        ## nn.AdaptiveAvgPool2D(1)
        stride = (net.shape[1], net.shape[2])
        kernel_size = (net.shape[1], net.shape[2])
        net = tf.layers.average_pooling2d(inputs=net, pool_size=kernel_size, strides=stride, padding='valid')

    axes = [1, 2]
    with tf.variable_scope('dense'):
        print(TAG, net)
        outputs = tf.reduce_mean(net, axes)
        print(TAG, outputs)
        outputs = tf.layers.dense(inputs=outputs, units=NUM_CLASSES)
        outputs = tf.identity(outputs, 'final_dense')
    return outputs


def alter(file, old_str, new_str):
    file_data = ""
    with open(file, "r", encoding="utf-8") as f:
        for line in f:
            if old_str in line:
                line = line.replace(old_str, new_str)
            file_data += line
    with open(file, "w", encoding="utf-8") as f:
        f.write(file_data)