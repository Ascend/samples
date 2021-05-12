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

AMCT_CAFFE sample of mnist model

"""
import os
import sys
import argparse
from pathlib import Path


MODEL_INPUT_BLOB_NAME = 'data'
MODEL_OUTPUT_BLOB_NAME = 'prob'

PATH = os.path.split(os.path.realpath(__file__))[0]
PATH = os.path.realpath(os.path.join(PATH, '..'))
TMP = os.path.join(PATH, 'tmp')
RESULT = os.path.join(PATH, 'results')
DATASET_PATH = os.path.join(PATH, 'data/mnist_data')
LMDB_DATASET_DIR = os.path.join(DATASET_PATH, 'mnist_test_lmdb')
BATCH_SIZE = 64
SCALE = 0.00390625


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Mnist demo')
    parser.add_argument('--cpu',
                        dest='cpu_mode',
                        help='Use CPU mode',
                        action='store_true')
    parser.add_argument('--gpu',
                        dest='gpu_id',
                        help='GPU device id to use',
                        default=None,
                        type=int)
    parser.add_argument('--model_file',
                        dest='model_file',
                        help='Specify the model file of caffe model.',
                        default='./model/mnist-deploy.prototxt',
                        type=str)
    parser.add_argument('--weights_file',
                        dest='weights_file',
                        help='Specify the weights file of caffe model.',
                        default='./model/mnist-model.caffemodel',
                        type=str)
    parser.add_argument('--caffe_dir', dest='caffe_dir',
                        help='Specify the dir of caffe',
                        default=None, type=str)
    parser.add_argument('--iterations',
                        dest='iterations',
                        help='Specify iterations of test',
                        default=100,
                        type=int)
    return parser.parse_args()


def args_check(args):
    """Check args"""
    # --weights_file
    if args.weights_file is None:
        raise RuntimeError('Must specify a caffe caffemodel file')
    mnist_weights_file = os.path.realpath(args.weights_file)
    if not Path(mnist_weights_file).exists():
        raise RuntimeError('Must specify a caffe caffemodel file')
    args.weights_file = mnist_weights_file
    # --model_file
    if args.model_file is None:
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    mnist_model_file = os.path.realpath(args.model_file)
    if not Path(mnist_model_file).exists():
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    args.model_file = mnist_model_file
    # --caffe_dir
    if args.caffe_dir is None:
        raise RuntimeError('Must specify a caffe framework dir')
    caffe_dir = os.path.realpath(args.caffe_dir)
    if not Path(caffe_dir).exists():
        raise RuntimeError('Must specify a caffe framework dir')
    pycaffe_file = os.path.join(caffe_dir, 'python/caffe/pycaffe.py')
    if not Path(pycaffe_file).exists():
        raise RuntimeError('Must make pycaffe before execute demo')
    if args.cpu_mode and args.gpu_id is not None:
        raise RuntimeError('Cannot run in CPU mode and GPU mode at same time.')


def add_path(path):
    """Add path to env"""
    if path not in sys.path:
        sys.path.insert(0, path)


QUANT_ARGS = parse_args()
args_check(QUANT_ARGS)
add_path(os.path.join(QUANT_ARGS.caffe_dir, 'python'))


import caffe # pylint: disable=E0401, C0413
import amct_caffe as amct # pylint: disable=E0401, C0413
import datasets # pylint: disable=C0413
import download_and_convert_mnist_dataset as mnist_data


def img_postprocess(probs, labels):
    """Do image post-process"""
    # calculate top1 and top5 accuracy
    top1_get = 0
    top5_get = 0
    prob_size = probs.shape[1]
    for index, label in enumerate(labels):
        top5_record = (probs[index, :].argsort())[prob_size - 5:prob_size]
        if label == top5_record[-1]:
            top1_get += 1
            top5_get += 1
        elif label in top5_record:
            top5_get += 1
    return float(top1_get) / len(labels), float(top5_get) / len(labels)


def do_benchmark_test(model_file, weights_file, iterations=150):
    """ Calc the accuracy on the lmdb dataset"""
    model_file = os.path.realpath(model_file)
    weights_file = os.path.realpath(weights_file)
    net = caffe.Net(model_file, weights_file, caffe.TEST)

    top1_total = 0
    lmdb_data = datasets.LMDBData(LMDB_DATASET_DIR)
    lmdb_data.set_scale(SCALE)
    for index in range(iterations):
        data, labels = lmdb_data.get_blobs(BATCH_SIZE)
        forward_kwargs = {MODEL_INPUT_BLOB_NAME: data}
        blobs_out = net.forward(**forward_kwargs)
        top1, _ = img_postprocess(blobs_out[MODEL_OUTPUT_BLOB_NAME], labels)
        top1_total += top1
        print('*****************iteration:{}******************'.format(index))
        print('top1_acc:{}'.format(top1))
    print('******final top1:{}'.format(top1_total / iterations))
    return top1_total / iterations


def main(args):
    """Main function"""
    # User model files
    model_file = args.model_file
    weights_file = args.weights_file

    # amct.set_cpu_mode() or set_gpu_mode() decides whether using CPU/GPU
    # to do weights calibration, but activation calibration is controled
    # by caffe APIs: caffe.set_mode_cpu() or set_mode_gpu().
    # Need to set amct mode before the whole calibration process, default
    # using CPU mode to do weights calibration.
    # amct.set_gpu_mode() does not set which GPU card to use. Users can set
    # GPU card in two ways:
    # 1) use pycaffe API set_device(gpu_id)
    # 2) use environment variable CUDA_VISIBLE_DEVICES
    if args.gpu_id is not None and not args.cpu_mode:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        amct.set_gpu_mode()
    else:
        caffe.set_mode_cpu()

    # Run pre model test
    ori_top1 = do_benchmark_test(model_file, weights_file, args.iterations)

    # Quantize configurations
    config_json_file = os.path.join(TMP, 'config.json')
    skip_layers = []
    batch_num = 2
    amct.create_quant_config(config_json_file, model_file, weights_file,
                             skip_layers, batch_num)

    # Phase0: Init amct task
    scale_offset_record_file = os.path.join(TMP, 'record.txt')
    graph = amct.init(config_json_file, model_file, weights_file,
                      scale_offset_record_file)

    # Phase1: do conv+bn+scale fusion, weights calibration and fake quant,
    #         insert quant and dequant layer
    modified_model_file = os.path.join(TMP, 'modified_model.prototxt')
    modified_weights_file = os.path.join(TMP, 'modified_model.caffemodel')
    amct.quantize_model(graph, modified_model_file, modified_weights_file)

    # Phase2: run caffe model to do activation calibration
    do_benchmark_test(modified_model_file, modified_weights_file, batch_num)

    # Phase3: save final model, one for caffe do fake quant test,
    #         one deploy model for GE
    result_path = os.path.join(RESULT, 'mnist')
    amct.save_model(graph, 'Both', result_path)

    # Phase4: do final fake quant model test
    fake_quant_model = os.path.join(RESULT, 'mnist_fake_quant_model.prototxt')
    fake_quant_weights = os.path.join(
        RESULT, 'mnist_fake_quant_weights.caffemodel')
    quant_top1 = do_benchmark_test(
        fake_quant_model, fake_quant_weights, args.iterations)
    print('[AMCT][INFO] mnist top1 before quantize is {}, after quantize ' \
        'is {}'.format(ori_top1, quant_top1))
    print('[AMCT][INFO]Run mnist sample with quantize success!')


if __name__ == '__main__':
    # Init mnist env
    mnist_data.download_mnist_dataset(DATASET_PATH)
    mnist_data.make_mnist_lmdb_dataset(QUANT_ARGS.caffe_dir, DATASET_PATH, DATASET_PATH)
    main(QUANT_ARGS)
