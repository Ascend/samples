#!/usr/bin/python3 # pylint: disable=C0103
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

AMCT_CAFFE sample of ResNet50 model for retrain feature

"""
import os
import sys
import argparse
import google.protobuf


TEST_SIZE = 50000
PATH = os.path.split(os.path.realpath(__file__))[0]
PATH = os.path.realpath(os.path.join(PATH, '..'))
PRE_MODEL = os.path.join(PATH, 'model')
TMP = os.path.join(PATH, 'tmp')
RESULT = os.path.join(PATH, 'results/retrain_results')


def parse_args():
    """parse user args"""
    parser = argparse.ArgumentParser(description='Retrain Demo')

    parser.add_argument('--cpu',
                        dest='cpu_mode',
                        help='Use CPU mode',
                        action='store_true')
    parser.add_argument('--gpu',
                        dest='gpu_id',
                        help='GPU device id to use',
                        default=None,
                        type=int)
    parser.add_argument('--caffe_dir',
                        dest='caffe_dir',
                        help='Specify the dir of caffe',
                        default=None,
                        type=str)
    parser.add_argument('--model_file',
                        dest='model_file',
                        help='Specify the model file of caffe model.',
                        default='./model/ResNet-50_retrain.prototxt',
                        type=str)
    parser.add_argument('--weights_file',
                        dest='weights_file',
                        help='Specify the weights file of caffe model.',
                        default='./model/ResNet-50-model.caffemodel',
                        type=str)
    parser.add_argument('--train_data',
                        dest='train_data',
                        help='Specify the train data for retrain.',
                        default=None,
                        type=str)
    parser.add_argument('--test_data',
                        dest='test_data',
                        help='Specify the test data for benchmark.',
                        default=None,
                        type=str)
    parser.add_argument('--train_batch',
                        dest='train_batch',
                        help='The number of samples in one training iteration.',
                        default=32,
                        type=int)
    parser.add_argument('--train_iter',
                        dest='train_iter',
                        help='The number of retrain iterations.',
                        default=1000,
                        type=int)
    parser.add_argument('--test_batch',
                        dest='test_batch',
                        help='The number of samples in one test iteration.',
                        default=1,
                        type=int)
    parser.add_argument('--test_iter',
                        dest='test_iter',
                        help='The number of test iterations.',
                        default=500,
                        type=int)

    return parser.parse_args()


def args_check_caffe_dir(args):
    """check args of caffe dir"""
    if args.caffe_dir is None:
        raise RuntimeError('Must specify a caffe framework directory!')
    args.caffe_dir = os.path.realpath(args.caffe_dir)

    if not os.path.exists(args.caffe_dir):
        raise RuntimeError('Must specify a caffe framework directory!')
    pycaffe_file = os.path.join(args.caffe_dir, 'python/caffe/pycaffe.py')

    if not os.path.exists(pycaffe_file):
        raise RuntimeError('Must make pycaffe before execute demo!')


def args_check(args):
    """check input args"""
    if args.model_file is None:
        raise RuntimeError('Must specify a caffe deploy prototxt file!')
    args.model_file = os.path.realpath(args.model_file)

    if not os.path.exists(args.model_file):
        raise RuntimeError('Must specify a caffe deploy prototxt file!')

    if args.weights_file is None:
        raise RuntimeError('Must specify a caffe caffemodel file!')
    args.weights_file = os.path.realpath(args.weights_file)

    if not os.path.exists(args.weights_file):
        raise RuntimeError('Must specify a caffe caffemodel file!')

    if args.train_data is None:
        raise RuntimeError('Must specify the train dataset path!')
    args.train_data = os.path.realpath(args.train_data)

    if not os.path.exists(args.train_data):
        raise RuntimeError('Must specify the train dataset path!')

    if args.test_data is None:
        raise RuntimeError('Must specify the test dataset path!')
    args.test_data = os.path.realpath(args.test_data)

    if not os.path.exists(args.test_data):
        raise RuntimeError('Must specify the test dataset path!')

    if args.cpu_mode and args.gpu_id is not None:
        raise RuntimeError('Cannot run in CPU mode and GPU mode at same time.')

    if args.test_iter * args.test_batch > TEST_SIZE:
        raise RuntimeError(
            'The total number of test sample has exceeded the test size, '
            'test_iter({}) * test_batch({}) = {} > {}'.format(
                args.test_iter,
                args.test_batch,
                args.test_iter * args.test_batch,
                TEST_SIZE))


def add_path(path):
    """Add path to env"""
    if path not in sys.path:
        sys.path.insert(0, path)


quant_args = parse_args()
args_check_caffe_dir(quant_args)
add_path(os.path.join(quant_args.caffe_dir, 'python'))


import caffe # pylint: disable=E0401, C0413
import amct_caffe # pylint: disable=E0401, C0413


def mkdir(name):
    """make dir"""
    if not os.access(name, os.F_OK):
        os.makedirs(name)


def test(model_file, weights_file, iterations):
    """test caffe model"""
    net = caffe.Net(model_file, weights_file, caffe.TEST)
    top_1 = 0
    top_5 = 0

    for _ in range(iterations):
        top_1 += net.forward()['acc/top-1']
        top_5 += net.forward()['acc/top-5']

    print('Top 1 accuracy =', top_1 / iterations)
    print('Top 5 accuracy =', top_5 / iterations)


def train(args, model_file, weights_file, solver_file):
    """train caffe model"""
    solver_param = caffe.proto.caffe_pb2.SolverParameter()
    solver_param.net = model_file
    solver_param.lr_policy = 'step'
    solver_param.base_lr = 0.0001
    solver_param.stepsize = 10
    solver_param.gamma = 0.1
    solver_param.momentum = 0.9
    solver_param.weight_decay = 0.0001
    solver_param.test_initialization = False
    solver_param.max_iter = args.train_iter
    solver_param.test_interval = args.train_iter
    solver_param.test_iter[:] = [args.test_iter]
    solver_param.snapshot = args.train_iter

    with open(solver_file, 'w') as file_open:
        file_open.write(str(solver_param))

    solver = caffe.SGDSolver(solver_file)
    solver.net.copy_from(weights_file)
    solver.solve()


def main(args):
    """main function"""
    args_check(args)

    # set_cpu_mode() or set_gpu_mode() decides whether using CPU or GPU
    # to retrain, default using GPU. set_gpu_mode() does not set which
    # GPU card to use. Users can set GPU card in two ways:
    # 1) use pycaffe API set_device(gpu_id)
    # 2) use environment variable CUDA_VISIBLE_DEVICES
    if args.gpu_id is not None and not args.cpu_mode:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        amct_caffe.set_gpu_mode()
    else:
        caffe.set_mode_cpu()

    # Modify the model based according to the specified dataset
    net = caffe.proto.caffe_pb2.NetParameter()

    with open(args.model_file, 'r') as file_open:
        google.protobuf.text_format.Merge(file_open.read(), net)

    net.layer[0].data_param.source = args.train_data
    net.layer[0].data_param.batch_size = args.train_batch
    net.layer[1].data_param.source = args.test_data
    net.layer[1].data_param.batch_size = args.test_batch

    with open(args.model_file, 'w') as file_open:
        file_open.write(google.protobuf.text_format.MessageToString(net))

    # Do the pre-training ResNet-50 model test.
    test(args.model_file, args.weights_file, args.test_iter)

    # Create the configuration file for ResNet-50 model retraining.
    config_file = os.path.join(TMP, 'config.json')
    amct_caffe.create_quant_retrain_config(
        config_file, args.model_file, args.weights_file)

    # Generate the ResNet-50 retraining model based on configuration
    # file, including '.prototxt' and '.caffemodel'.
    modified_model_file = os.path.join(TMP, 'modified_model.prototxt')
    modified_weights_file = os.path.join(TMP, 'modified_model.caffemodel')
    scale_offset_record_file = os.path.join(TMP, 'record.txt')
    amct_caffe.create_quant_retrain_model(
        args.model_file, args.weights_file, config_file, modified_model_file,
        modified_weights_file, scale_offset_record_file)

    # Retrain the model.
    solver_file = os.path.join(TMP, 'solver.prototxt')
    train(args, modified_model_file, modified_weights_file, solver_file)

    # Export 'deploy' and 'fakequant' models based on the trained model.
    retrained_weights_file = os.path.join(
        TMP, 'solver_iter_{}.caffemodel'.format(args.train_iter))
    result_path = os.path.join(RESULT, 'ResNet50')
    amct_caffe.save_quant_retrain_model(
        modified_model_file, retrained_weights_file, 'Both', result_path,
        scale_offset_record_file, config_file)

    # Do the retrained ResNet-50 model test.
    fake_quant_model = os.path.join(
        RESULT, 'ResNet50_fake_quant_model.prototxt')
    fake_quant_weights = os.path.join(
        RESULT, 'ResNet50_fake_quant_weights.caffemodel')
    test(fake_quant_model, fake_quant_weights, args.test_iter)

    # Modify the deploy model to adapt ATC toolkit
    deploy_quant_model = os.path.join(RESULT, 'ResNet50_deploy_model.prototxt')

    net = caffe.proto.caffe_pb2.NetParameter()
    with open(deploy_quant_model, 'r') as file_open:
        google.protobuf.text_format.Merge(file_open.read(), net)

    input_layer = net.layer[0]
    input_layer.Clear()
    input_layer.name = 'data'
    input_layer.type = 'Input'
    input_layer.top[:] = ['data']
    input_layer.input_param.shape.add().dim[:] = [1, 3, 224, 224]
    del net.layer[-1]
    del net.layer[-1]
    del net.layer[-1]

    atc_model = os.path.join(RESULT, 'ResNet50_atc_model.prototxt')
    with open(atc_model, 'w') as file_open:
        file_open.write(google.protobuf.text_format.MessageToString(net))


if __name__ == '__main__':
    mkdir(PRE_MODEL)
    mkdir(TMP)
    mkdir(RESULT)
    main(quant_args)
