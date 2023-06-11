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

import os
from copy import deepcopy
from operations import OPS

TAG = 'NAS_Bench_201:' + os.path.basename(__file__)


class InferCell():
    def __init__(self, inputs, genotype, C_out, stride, is_training=True, data_format='channels_last'):
        self.is_training = is_training
        self.data_format = data_format
        self.stride = stride
        self.node0 = inputs
        self.node1 = None
        self.node2 = None
        self.node3 = None
        self.out_dim = C_out
        self.genotype = deepcopy(genotype)
        self.node_IN = []
        self.node_IX = []

    def cal_node_1(self, node0, node0_op):
        op = OPS[node0_op](is_training=self.is_training, data_format=self.data_format)
        node1 = op.build(inputs=node0, channels=self.out_dim, stride=self.stride)
        return node1

    def cal_node_2(self, node0, node0_op, node1, node1_op):
        op_from_node0 = OPS[node0_op](is_training=self.is_training, data_format=self.data_format)
        temp_data_from_node0 = op_from_node0.build(inputs=node0, channels=self.out_dim, stride=self.stride)

        op_from_node1 = OPS[node1_op](is_training=self.is_training, data_format=self.data_format)
        temp_data_from_node1 = op_from_node1.build(inputs=node1, channels=self.out_dim, stride=self.stride)

        node2 = temp_data_from_node0 + temp_data_from_node1
        return node2

    def cal_node_3(self, node0, node0_op, node1, node1_op, node2, node2_op):
        op_from_node0 = OPS[node0_op](is_training=self.is_training, data_format=self.data_format)
        temp_data_from_node0 = op_from_node0.build(inputs=node0, channels=self.out_dim, stride=self.stride)

        op_from_node1 = OPS[node1_op](is_training=self.is_training, data_format=self.data_format)
        temp_data_from_node1 = op_from_node1.build(inputs=node1, channels=self.out_dim, stride=self.stride)

        op_from_node2 = OPS[node2_op](is_training=self.is_training, data_format=self.data_format)
        temp_data_from_node2 = op_from_node2.build(inputs=node2, channels=self.out_dim, stride=self.stride)

        node3 = temp_data_from_node0 + temp_data_from_node1 + temp_data_from_node2
        return node3

    def build(self):
        count = 0
        for i in range(1, len(self.genotype)):
            node_info = self.genotype[i - 1]
            print(TAG + '.info', node_info)
            if count == 0:
                self.node1 = self.cal_node_1(self.node0, node_info[0][0])
            elif count == 1:
                self.node2 = self.cal_node_2(self.node0, node_info[0][0], self.node1, node_info[1][0])
            elif count == 2:
                self.node3 = self.cal_node_3(self.node0, node_info[0][0], self.node1, node_info[1][0], self.node2, node_info[2][0])
            count = count + 1
        return self.node3