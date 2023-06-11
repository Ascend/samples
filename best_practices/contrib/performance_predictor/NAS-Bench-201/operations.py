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

import tensorflow as tf
import os

TAG = 'NAS_Bench_201' + os.path.basename(__file__)

VALID_DATA_FORMATS = frozenset(['channels_last', 'channels_first'])


def ReLUConvBN(inputs, conv_size, channels, stride, padding, is_training, data_format, affine=True):
    if data_format == 'channels_last':
        axis = 3
    elif data_format == 'channels_first':
        axis = 1
    else:
        raise ValueError('invalid data_format')

    net = tf.nn.relu(inputs)
    print(TAG, net)
    net = tf.layers.conv2d(
        inputs=net,
        filters=channels,
        kernel_size=conv_size,
        strides=stride,
        padding=padding,
        use_bias=not affine,
        kernel_initializer=tf.variance_scaling_initializer()
    )
    print(TAG, net)
    net = tf.layers.batch_normalization(
        inputs=net,
        axis=axis,
        training=is_training
    )
    return net


class BaseOp(object):
    def __init__(self, is_training, data_format='channels_last'):
        self.is_training = is_training
        if data_format.lower() not in VALID_DATA_FORMATS:
            raise ValueError('invalid data_format')
        self.data_format = data_format.lower()
        self.out_dim = -1

    def build(self, inputs, channels, stride):
        pass


class ReluConv3x3BN(BaseOp):
    def build(self, inputs, channels, stride):
        net = ReLUConvBN(inputs, 3, channels, stride, 'same', self.is_training, self.data_format)
        return net


class ReluConv1x1BN(BaseOp):
    def build(self, inputs, channels, stride):
        net = ReLUConvBN(inputs, 1, channels, stride, 'valid', self.is_training, self.data_format)
        return net


class AvgPooling(BaseOp):
    def build(self, inputs, channels, stride):
        self.out_dim = channels
        in_channels = inputs.get_shape()[-1]
        print(TAG, inputs)
        if in_channels == self.out_dim:
            preprocess = None
        else:
            preprocess = ReLUConvBN(inputs=inputs, conv_size=1, channels=self.out_dim, stride=1, padding='valid', is_training=self.is_training, data_format=self.data_format)
        if preprocess is not None:
            inputs = preprocess
        with tf.variable_scope('AvgPool3_3'):
            net = tf.layers.average_pooling2d(
                inputs=inputs,
                pool_size=(3, 3),
                strides=stride,
                padding='same',
                data_format=self.data_format
            )
        print(TAG, net)
        return net


class Zero(BaseOp):
    def build(self, inputs, channels, stride):
        in_channels = inputs.get_shape()[-1]
        if in_channels == channels:
            if stride == 1:
                return tf.multiply(inputs, 0.0)
            else:
                return inputs[:, :, :: stride, :: stride].mul(0.0)
        else:
            shape = list(inputs.shape)
            shape[1] = channels
            zeros = tf.zeros(shape, dtype=inputs.dtype)
            return zeros


class SkipConnection(BaseOp):
    def build(self, inputs, channels, stride):
        in_channels = inputs.get_shape()[-1]
        self.out_dim = channels
        if stride == 1 and in_channels == self.out_dim:
            return inputs
        else:
            with tf.variable_scope('FactorizedReduce'):
                print(TAG, 'FactorizedReduce')
                paddings = tf.constant([[0, 1, ], [0, 1]])
                channel_list = [self.out_dim // 2, self.out_dim - self.out_dim // 2]
                if stride == 2:
                    net_v1 = tf.nn.relu(inputs)
                    net_v2 = tf.pad(inputs, paddings=paddings)
                    net_v1 = tf.layers.conv2d(
                        inputs=net_v1,
                        filters=channel_list[0],
                        kernel_size=1,
                        strides=stride,
                        padding='valid',
                        use_bias=False,
                        kernel_initializer=tf.variance_scaling_initializer(),
                        data_format=self.data_format
                    )
                    net_v2 = tf.layers.conv2d(
                        inputs=net_v2[:, :, 1:, 1:],
                        filters=channel_list[1],
                        kernel_size=1,
                        strides=stride,
                        padding='valid',
                        use_bias=False,
                        kernel_initializer=tf.variance_scaling_initializer(),
                        data_format=self.data_format
                    )
                    net = tf.concat([net_v1, net_v2], axis=1)
                else:
                    net = tf.layers.conv2d(
                        inputs=inputs,
                        filters=self.out_dim,
                        strides=stride,
                        padding='valid',
                        use_bias=False,
                        kernel_initializer=tf.variance_scaling_initializer(),
                        data_format=self.data_format
                    )
                net = tf.layers.batch_normalization(
                    inputs=net,
                    axis=3,
                    training=self.is_training
                )
            return net


class ResNetBasicblock(BaseOp):
    def build(self, inputs, channels, stride):
        assert stride == 1 or stride == 2, "invalid stride {:}".format(stride)
        self.out_dim = channels
        in_channels = inputs.get_shape()[-1]
        net = ReLUConvBN(inputs, 3, self.out_dim, stride, 'same', self.is_training, self.data_format)
        net = ReLUConvBN(net, 3, self.out_dim, 1, 'same', self.is_training, self.data_format)
        if stride == 2:
            downsample = tf.layers.average_pooling2d(
                inputs=inputs,
                pool_size=(2, 2),
                strides=2,
                padding='valid'
            )
            downsample = tf.layers.conv2d(
                inputs=downsample,
                filters=self.out_dim,
                kernel_size=1,
                strides=1,
                padding='valid',
                use_bias=False,
                kernel_initializer=tf.variance_scaling_initializer()
            )
        elif in_channels == self.out_dim:
            downsample = ReLUConvBN(inputs, 1, self.out_dim, 1, 'valid', self.is_training, self.data_format)
        else:
            downsample = None

        if downsample is not None:
            residual = downsample
        else:
            residual = inputs
        return net + residual


OPS = {
    "none": Zero,
    "avg_pool_3x3": AvgPooling,
    "nor_conv_3x3": ReluConv3x3BN,
    "nor_conv_1x1": ReluConv1x1BN,
    "skip_connect": SkipConnection
}