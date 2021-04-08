#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

AMCT_MINDSPORE sample of ResNet50
"""

import os
import argparse
import random
import numpy as np

from mindspore import dataset as de # pylint: disable=E0401
from mindspore.nn.loss import SoftmaxCrossEntropyWithLogits # pylint: disable=E0401
from mindspore.train.serialization import load_checkpoint # pylint: disable=E0401
from mindspore.train.serialization import load_param_into_net # pylint: disable=E0401
from mindspore.train import Model # pylint: disable=E0401
from mindspore import context # pylint: disable=E0401
from amct_mindspore.quantize_tool import create_quant_config # pylint: disable=import-error
from amct_mindspore.quantize_tool import quantize_model # pylint: disable=import-error
from amct_mindspore.quantize_tool import save_model # pylint: disable=import-error
from resnet import resnet50 # pylint: disable=import-error, no-name-in-module
from dataset import create_dataset1 as create_dataset # pylint: disable=import-error, no-name-in-module

PARSER = argparse.ArgumentParser(description='Resnet50 Image classification base on CIFAR-10 dataset')

PARSER.add_argument('--dataset_path',
                    type=str,
                    default=None,
                    help='Dataset path')
PARSER.add_argument('--checkpoint_path',
                    type=str,
                    default=None,
                    help='checkpoint path')


ARGS_OPT = PARSER.parse_args()

random.seed(1)
np.random.seed(1)
de.config.set_seed(1)


def quant_resnet50(network, dataset, loss, input_data):
    """quantize the resnet50 """

    # step2: creat the quant config json file
    create_quant_config('./config.json', network, input_data)

    # step3: do some network modification and return the modified network
    calibration_network = quantize_model('./config.json', network, input_data)
    calibration_network.set_train(False)

    # step4: perform the evaluation of network to do activation calibration
    model = Model(calibration_network,
                  loss_fn=loss,
                  metrics={'top_1_accuracy', 'top_5_accuracy'})

    _ = model.eval(dataset, dataset_sink_mode=False)

    # step5: export the air file
    save_model('results/resnet50_quant', calibration_network, input_data)
    print("[INFO] the quantized AIR file has been stored at: \n {}".format('results/resnet50_quant.air'))


def main():
    """ main function """
    os.environ["DEVICE_NUM"] = "1"
    os.environ["RANK_ID"] = "0"
    target = 'Ascend'
    context.set_context(mode=context.GRAPH_MODE, device_target=target)

    # step1: create_dataset for evaluation, prepare the input data
    # and initialize the network, load the pretrained checkpoint to the network.
    # Ensure that the network before quant_resnet50 is proper functioning.
    dataset = create_dataset(dataset_path=ARGS_OPT.dataset_path,
                             do_train=False,
                             batch_size=32,
                             target=target)

    loss = SoftmaxCrossEntropyWithLogits(sparse=True, reduction='mean')

    dataset = dataset.take(1)
    input_shape = [32, 3, 224, 224]


    class_num = 10
    input_data = np.random.uniform(0.0, 1.0, size=input_shape).astype(np.float32)
    network = resnet50(class_num)
    param_dict = load_checkpoint(ARGS_OPT.checkpoint_path)
    load_param_into_net(network, param_dict)
    network.set_train(False)

    quant_resnet50(network, dataset, loss, input_data)


if __name__ == "__main__":
    main()
