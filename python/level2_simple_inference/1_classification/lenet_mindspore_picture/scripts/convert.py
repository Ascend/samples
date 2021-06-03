"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-01-29 14:04:45
"""
from mindspore.train.serialization import load_checkpoint, save_checkpoint, export
from src.lenet import LeNet5
import numpy as np
from mindspore import Tensor
network = LeNet5()
load_checkpoint("./checkpoint_lenet-1_1875.ckpt", network)
input_data = np.random.uniform(0.0, 1.0, size = [1, 1, 32, 32]).astype(np.float32)
export(network, Tensor(input_data), file_name = './mnist', file_format = 'AIR') 