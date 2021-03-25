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

AMCT_MINDSPORE sample of ResNet50 quant aware training
"""

import os
import sys
import argparse
import random
import ast
import numpy as np

import mindspore # pylint: disable=E0401
from mindspore import dataset as de # pylint: disable=E0401
from mindspore.nn.optim.momentum import Momentum # pylint: disable=E0401
from mindspore.train import Model # pylint: disable=E0401
from mindspore import context # pylint: disable=E0401
from mindspore import Tensor # pylint: disable=E0401
from mindspore.context import ParallelMode # pylint: disable=E0401
from mindspore.train.callback import ModelCheckpoint # pylint: disable=E0401
from mindspore.train.callback import CheckpointConfig # pylint: disable=E0401
from mindspore.train.callback import LossMonitor # pylint: disable=E0401
from mindspore.train.callback import TimeMonitor # pylint: disable=E0401
from mindspore.train.callback import Callback # pylint: disable=E0401
from mindspore.nn.loss import SoftmaxCrossEntropyWithLogits # pylint: disable=E0401
from mindspore.train.loss_scale_manager import FixedLossScaleManager # pylint: disable=E0401
from mindspore.train.serialization import load_checkpoint # pylint: disable=E0401
from mindspore.train.serialization import load_param_into_net # pylint: disable=E0401
from mindspore.communication.management import init # pylint: disable=E0401
from mindspore.communication.management import get_rank # pylint: disable=E0401

import amct_mindspore as amct # pylint: disable=import-error
from amct_mindspore.initializer import UlqInitializer # pylint: disable=import-error

from lr_generator import get_lr # pylint: disable=import-error, no-name-in-module
from resnet import resnet50 as resnet # pylint: disable=import-error, no-name-in-module
# use the quant retrain config to retrain resnet50 for cifar-10 dataset
from retrain_config import CONFIG_QUANT as config
from dataset import create_dataset1 as create_dataset # pylint: disable=import-error, no-name-in-module

CUR_DIR = os.path.split(os.path.realpath(__file__))[0]


PARSER = argparse.ArgumentParser(description='Image classification')
PARSER.add_argument('--net', type=str, default='resnet50', help='Resnet Model, [resnet50]')
PARSER.add_argument('--dataset', type=str, default='cifar10', help='Dataset, [cifar10]')
PARSER.add_argument('--run_distribute', type=ast.literal_eval, default=False, help='Run distribute')
PARSER.add_argument('--device_num', type=int, default=1, help='Device num.')

PARSER.add_argument('--device_target', type=str, default='Ascend', help='Device target')
PARSER.add_argument('--pre_trained', type=str, default='', help='Pretrained checkpoint path')
PARSER.add_argument('--air', type=str, default='resnet50_quant_retrain', help='the export air file name')
PARSER.add_argument('--original', action='store_true', help='whether test original resnet50')
PARSER.add_argument('--eval_dataset', type=str, default='', help='eval dataset path')
PARSER.add_argument('--train_dataset', type=str, default='', help='train dataset path')
ARGS_OPT = PARSER.parse_args()

random.seed(1)
np.random.seed(1)
de.config.set_seed(1)



class EvalCallBack(Callback): # pylint: disable=R0903
    """Precision verification using callback function."""
    # define the operator required
    def __init__(self, models, eval_dataset, eval_per_epochs, epochs_per_eval):
        super().__init__()
        self.models = models
        self.eval_dataset = eval_dataset
        self.eval_per_epochs = eval_per_epochs
        self.epochs_per_eval = epochs_per_eval

    # define operator function in epoch end
    def epoch_end(self, run_context):
        """ when epoch end"""
        cb_param = run_context.original_args()
        cur_epoch = cb_param.cur_epoch_num
        if cur_epoch % self.eval_per_epochs == 0:
            eval_result = self.models.eval(self.eval_dataset, dataset_sink_mode=False)
            self.epochs_per_eval["epoch"].append(cur_epoch)
            self.epochs_per_eval["acc"].append(eval_result["top_1_accuracy"])
            print("***************** test in epoch {} *****************".format(cur_epoch))
            print(eval_result)


def test_original_resnet50():
    """ evaluate the original resnet50"""
    dataset = create_dataset(dataset_path=ARGS_OPT.eval_dataset, do_train=False,
                             batch_size=config.batch_size, # pylint: disable=no-member
                             target=ARGS_OPT.device_target)
    network = resnet(10)
    network.set_train(False)
    param_dict = load_checkpoint(ARGS_OPT.pre_trained)
    load_param_into_net(network, param_dict)
    loss = SoftmaxCrossEntropyWithLogits(sparse=True, reduction='mean')
    model = Model(network, loss_fn=loss, metrics={'top_1_accuracy', 'top_5_accuracy'})
    res = model.eval(dataset)
    print("result for original resnet50:", res, "ckpt=", ARGS_OPT.pre_trained)


def calibration():
    """ do the calibration to get the scale offset record file"""
    dataset = create_dataset(dataset_path=ARGS_OPT.eval_dataset, do_train=False,
                             batch_size=config.batch_size, # pylint: disable=no-member
                             target=ARGS_OPT.device_target)
    dataset = dataset.take(1)
    loss = SoftmaxCrossEntropyWithLogits(sparse=True, reduction='mean')

    network = resnet(10)
    network.set_train(False)
    param_dict = load_checkpoint(ARGS_OPT.pre_trained)
    load_param_into_net(network, param_dict)
    input_data = np.random.uniform(0.0, 1.0, size=[32, 3, 224, 224]).astype(np.float32)
    config_file = os.path.join(CUR_DIR, './config.json')
    amct.create_quant_config(config_file, network, input_data)
    calibration_network = amct.quantize_model(config_file, network, input_data)

    model = Model(calibration_network, loss_fn=loss, metrics={'top_1_accuracy', 'top_5_accuracy'})
    _ = model.eval(dataset)
    amct.save_model('./resnet50_quant_calibration', calibration_network, input_data)


def quant_retrain(ckpt_save_dir, target):
    """ quant aware traing the resnet50"""
    context.set_context(mode=context.GRAPH_MODE, device_target='Ascend', save_graphs=False)
    network = resnet(10)
    network.set_train(True)
    param_dict = load_checkpoint(ARGS_OPT.pre_trained)
    load_param_into_net(network, param_dict)
    input_data = np.random.uniform(0.0, 1.0, size=[32, 3, 224, 224]).astype(np.float32)

    # using the calibration result to initialize the retrain learnable parameters
    config_file = os.path.join(CUR_DIR, './retrain_quant_config.json')
    initializer = UlqInitializer('./amct_dump/calibration_record.txt')

    # 1. create retrain config file
    amct.create_quant_retrain_config(config_file, network, input_data)

    # 2. create the retrain network, insert ActUlq and WeightArq cell
    retrain_network = amct.create_quant_retrain_model(config_file,
                                                      network,
                                                      initializer,
                                                      input_data)

    # 3. retrain the network to update clip_min / clip_max and other learnable parameters
    train(retrain_network, ckpt_save_dir, target)

    # 4. save the retrained network to AIR file formate
    retrain_network.to_float(mindspore.float32)
    amct.save_quant_retrain_model(config_file,
                                  ARGS_OPT.air,
                                  retrain_network,
                                  input_data)


def train(net, ckpt_save_dir, target): # pylint: disable=too-many-locals
    """ train the network"""
    # create dataset
    train_dataset = create_dataset(dataset_path=ARGS_OPT.train_dataset, do_train=True, repeat_num=1,
                                   batch_size=config.batch_size, target=target) # pylint: disable=no-member
    step_size = train_dataset.get_dataset_size()
    # init lr
    learning_rate = get_lr(lr_init=config.lr_init, lr_end=config.lr_end, # pylint: disable=no-member
                           lr_max=config.lr_max, warmup_epochs=config.warmup_epochs, # pylint: disable=no-member
                           total_epochs=config.epoch_size, # pylint: disable=no-member
                           steps_per_epoch=step_size, lr_decay_mode=config.lr_decay_mode) # pylint: disable=no-member
    learning_rate = Tensor(learning_rate)

    # define opt
    decayed_params = []
    no_decayed_params = []
    for param in net.trainable_params():
        if 'beta' not in param.name and 'gamma' not in param.name and 'bias' not in param.name:
            decayed_params.append(param)
        else:
            no_decayed_params.append(param)

    group_params = [{'params': decayed_params, 'weight_decay': config.weight_decay}, # pylint: disable=no-member
                    {'params': no_decayed_params},
                    {'order_params': net.trainable_params()}]
    opt = Momentum(group_params, learning_rate,
                   config.momentum, loss_scale=config.loss_scale) # pylint: disable=no-member

    # define loss, model
    loss = SoftmaxCrossEntropyWithLogits(sparse=True, reduction='mean')
    loss_scale = FixedLossScaleManager(config.loss_scale, drop_overflow_update=False) # pylint: disable=no-member
    model = Model(net,
                  loss_fn=loss,
                  optimizer=opt,
                  loss_scale_manager=loss_scale,
                  metrics={'top_1_accuracy', 'top_5_accuracy'},
                  amp_level="O2",
                  keep_batchnorm_fp32=False)

    # define callbacks
    time_cb = TimeMonitor(data_size=step_size)
    loss_cb = LossMonitor()
    callbacks = [time_cb, loss_cb]
    if config.save_checkpoint: # pylint: disable=no-member
        config_ck = CheckpointConfig(
            save_checkpoint_steps=config.save_checkpoint_epochs * step_size, # pylint: disable=no-member
            keep_checkpoint_max=config.keep_checkpoint_max) # pylint: disable=no-member
        ckpt_cb = ModelCheckpoint(prefix="resnet", directory=ckpt_save_dir, config=config_ck)
        callbacks += [ckpt_cb]

    # define the eval call back
    epochs_per_eval = {"epoch": [], "acc": []}
    if not ARGS_OPT.run_distribute:
        eval_dataset = create_dataset(dataset_path=ARGS_OPT.eval_dataset,
                                      do_train=False,
                                      batch_size=config.batch_size, # pylint: disable=no-member
                                      target=target)
        eval_cb = EvalCallBack(model, eval_dataset, 1, epochs_per_eval)
        callbacks.append(eval_cb)

    # start training the qunat aware training network
    model.train(config.epoch_size, train_dataset, callbacks=callbacks, # pylint: disable=no-member
                sink_size=train_dataset.get_dataset_size(),
                dataset_sink_mode=False)
    if not ARGS_OPT.run_distribute:
        print("***************** evaluation results of training process ***************** ")
        print(epochs_per_eval)


def main():
    """" main function"""
    # set up the DUMP_AMCT_RECORD=1 to dump the scale offset record file;
    # the record file is needed for retrain initialization
    os.environ['DUMP_AMCT_RECORD'] = '1'
    if ARGS_OPT.original:
        test_original_resnet50()
        sys.exit()

    if not ARGS_OPT.run_distribute:
        context.set_context(mode=context.GRAPH_MODE, device_target=ARGS_OPT.device_target)

        # do the calibration to obtain the scale_offset_record file, which is needed for
        # quant aware training algorithm initialization
        calibration()

    # do the calibration to obtain the scale_offset_record file, which is needed for
    # quant aware training algorithm initialization
    print("The training config:")
    print(config)

    target = ARGS_OPT.device_target
    ckpt_save_dir = config.save_checkpoint_path # pylint: disable=no-member
    _ = os.path.realpath(os.path.join(CUR_DIR, ckpt_save_dir))
    if not os.path.exists(ckpt_save_dir):
        os.makedirs(ckpt_save_dir)

    # init context
    if ARGS_OPT.run_distribute:
        if target == "Ascend":
            device_id = int(os.getenv('DEVICE_ID'))
            context.set_context(device_id=device_id, enable_auto_mixed_precision=True)
            context.set_auto_parallel_context(device_num=ARGS_OPT.device_num,
                                              parallel_mode=ParallelMode.DATA_PARALLEL,
                                              gradients_mean=True)
            if ARGS_OPT.net == "resnet50":
                context.set_auto_parallel_context(all_reduce_fusion_config=[85, 160])
            else:
                raise RuntimeError("Only support Resnet-50, not support other model now.")
            init()
        # other target
        else:
            raise RuntimeError("Only support Ascend device target, not support other platform now.")
        ckpt_save_dir = config.save_checkpoint_path + "ckpt_" + str(get_rank()) + "/" # pylint: disable=no-member

    # start to quant aware training the resnet50
    quant_retrain(ckpt_save_dir, target)



if __name__ == "__main__":
    main()
