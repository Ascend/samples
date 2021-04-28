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

AMCT_CAFFE sample of accuracy_based_auto_calibration based on MobileNet V2

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys
import argparse
from pathlib import Path
import numpy as np

import cv2 # pylint: disable=E0401
import datasets

MODEL_INPUT_BLOB_NAME = 'data'
MODEL_OUTPUT_BLOB_NAME = 'prob'

PATH = os.path.split(os.path.realpath(__file__))[0]
PATH = os.path.realpath(os.path.join(PATH, '..'))
TMP = os.path.join(PATH, 'tmp')
RESULT = os.path.join(PATH, 'results')
BATCH_SIZE = 32
SCALE = 0.017
CROP_SIZE = 224
MEAN_FILE = None
MEAN_VALUE = [103.94, 116.78, 123.68]

DATA_DIR = os.path.join(PATH, 'data/images')
LABEL_FILE = os.path.join(DATA_DIR, 'image_label.txt')

# Need to specify the dir of caffe and dataset (ImageNet)
CAFFE_DIR = ''
LMDB_DATASET_DIR = ''
CALIBRATION_BATCH_NUM = 2


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Mobilenet_v2 demo')
    parser.add_argument('--model_file', dest='model_file',
                        help='Specify the model file of caffe model.',
                        default='./model/mobilenet_v2_deploy.prototxt',
                        type=str)
    parser.add_argument('--weights_file', dest='weights_file',
                        help='Specify the weights file of caffe model.',
                        default="./model/mobilenet_v2.caffemodel",
                        type=str)
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
                        default=None, type=int)
    parser.add_argument('--iterations', dest='iterations',
                        help='Specify iterations of test',
                        default=1000, type=int)
    parser.add_argument('--caffe_dir', dest='caffe_dir',
                        help='Specify the dir of caffe',
                        default=CAFFE_DIR,
                        type=str)
    parser.add_argument('--pre_test', dest='pre_test',
                        help='Do test with amct caffe calibration or not',
                        action='store_false')
    parser.add_argument('--dataset',
                        dest='dataset',
                        help='The path of benchmark dataset.',
                        default=LMDB_DATASET_DIR,
                        type=str)
    args = parser.parse_args()
    return args


def args_check(args):
    """check args"""
    # --model_file
    if args.model_file is None:
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    model_file = os.path.realpath(args.model_file)
    if not Path(model_file).exists():
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    # --weights_file
    if args.weights_file is None:
        raise RuntimeError('Must specify a caffe caffemodel file')
    weights_file = os.path.realpath(args.weights_file)
    if not Path(weights_file).exists():
        raise RuntimeError('Must specify a caffe caffemodel file')
    # --iterations
    if args.iterations > 1500:
        raise RuntimeError('Max iterations on sample dataset is 1500')


def args_check_caffe_dir(args):
    """check args of caffe dir"""
    if args.caffe_dir is None:
        raise RuntimeError('Must specify a caffe framework dir')
    caffe_dir = os.path.realpath(args.caffe_dir)
    if not Path(caffe_dir).exists():
        raise RuntimeError('Must specify a caffe framework dir')
    caffe_exec_bin = os.path.join(caffe_dir, 'build/tools/caffe')
    if not Path(caffe_exec_bin).exists():
        raise RuntimeError('Must make caffe before execute demo')
    pycaffe_file = os.path.join(caffe_dir, 'python/caffe/pycaffe.py')
    if not Path(pycaffe_file).exists():
        raise RuntimeError('Must make pycaffe before execute demo')


def add_path(path):
    """Add path to env"""
    if path not in sys.path:
        sys.path.insert(0, path)


QUANT_ARGS = parse_args()
args_check(QUANT_ARGS)
args_check_caffe_dir(QUANT_ARGS)
add_path(os.path.join(QUANT_ARGS.caffe_dir, 'python'))

import caffe # pylint: disable=E0401, C0413


import amct_caffe as amct # pylint: disable=E0401, C0413
from amct_caffe.common.auto_calibration import \
    AutoCalibrationEvaluatorBase # pylint: disable=E0401, C0413


def get_blobs_from_im(data_dir, imgs, batch_size):
    """Read image files to blobs [3, 256, 256]"""
    if batch_size != len(imgs):
        raise RuntimeError('batch_size:{} != len(imgs):{}'.format(
            batch_size, len(imgs)))

    blobs_data = np.zeros((batch_size, 3, 256, 256), np.uint8)
    for index in range(batch_size):
        im_file = os.path.join(data_dir, imgs[index])
        im_data = cv2.imread(im_file)
        im_data = cv2.resize(
            im_data, (256, 256), interpolation=cv2.INTER_CUBIC)
        im_data = im_data.swapaxes(0, 2)
        im_data = im_data.swapaxes(1, 2)
        blobs_data[index, :, :, :] = im_data

    return blobs_data


def get_labels_from_txt():
    """Read all images' name and label from label_file"""
    images = []
    labels = []
    with open(LABEL_FILE, 'r') as label_file:
        lines = label_file.readlines()
        for line in lines:
            images.append(line.split(' ')[0])
            labels.append(int(line.split(' ')[1]))
    return images, labels


def img_preprocess(blobs_data, mean_value, crop_size):
    """Do image data pre-process"""
    # crop image[height, width] to [crop_size, crop_size]
    height = blobs_data.shape[2]
    width = blobs_data.shape[3]
    h_off = int((height - crop_size) / 2)
    w_off = int((width - crop_size) / 2)
    crop_data = blobs_data[:, :, h_off:(height - h_off), w_off:(width - w_off)]
    # trans uint8 image data to float
    crop_data = crop_data.astype(np.float32, copy=False)
    # do channel-wise reduce mean value
    for channel in range(crop_data.shape[1]):
        crop_data[:, channel, :, :] -= mean_value[channel]
    # mutiply the scale value
    crop_data *= SCALE
    return crop_data


def img_postprocess(probs, labels):
    """Do image post-process"""
    # calculate top1 and top5 accuracy
    top1_get = 0
    top5_get = 0
    if len(probs.shape) == 4:
        probs = probs.reshape((probs.shape[0], probs.shape[1]))
    prob_size = probs.shape[1]
    for index, label in enumerate(labels):
        top5_record = (probs[index, :].argsort())[prob_size - 5:prob_size]
        if label == top5_record[-1]:
            top1_get += 1
            top5_get += 1
        elif label in top5_record:
            top5_get += 1
    return float(top1_get) / len(labels), float(top5_get) / len(labels)


def run_caffe_model(model_file, weights_file, iterations):
    """run caffe model forward"""
    net = caffe.Net(model_file, weights_file, caffe.TEST)

    top1_total = 0
    top5_total = 0
    images, labels = get_labels_from_txt()
    for iter_num in range(iterations):
        blobs_data = get_blobs_from_im(
            DATA_DIR,
            images[iter_num * BATCH_SIZE:(iter_num + 1) * BATCH_SIZE],
            BATCH_SIZE)
        blobs_data = img_preprocess(blobs_data, [104, 117, 123], 224)
        forward_kwargs = {MODEL_INPUT_BLOB_NAME: blobs_data}
        blobs_out = net.forward(**forward_kwargs)
        top1, top5 = img_postprocess(
            blobs_out[MODEL_OUTPUT_BLOB_NAME],
            labels[iter_num * BATCH_SIZE:(iter_num + 1) * BATCH_SIZE])
        top1_total += top1
        top5_total += top5
        print('****************iteration:{}*****************'.format(iter_num))
        print('top1_acc:{}'.format(top1))
        print('top5_acc:{}'.format(top5))
    print('******final top1:{}'.format(top1_total / iterations))
    print('******final top5:{}'.format(top5_total / iterations))


def do_benchmark_test(args, model_file, weights_file, iterations=1000):
    """ Calc the accuracy on the lmdb dataset"""
    net = caffe.Net(model_file, weights_file, caffe.TEST)

    top1_total = 0
    top5_total = 0
    lmdb_data = datasets.LMDBData(args.dataset)
    lmdb_data.set_scale(SCALE)
    lmdb_data.set_crop_size(CROP_SIZE)
    if MEAN_FILE is not None:
        lmdb_data.set_mean_file(MEAN_FILE)
    else:
        lmdb_data.set_mean_value(MEAN_VALUE)
    for index in range(iterations):
        data, labels = lmdb_data.get_blobs(BATCH_SIZE)
        forward_kwargs = {MODEL_INPUT_BLOB_NAME: data}
        blobs_out = net.forward(**forward_kwargs)
        top1, top5 = img_postprocess(blobs_out[MODEL_OUTPUT_BLOB_NAME], labels)
        top1_total += top1
        top5_total += top5
        print('*****************iteration:{}******************'.format(index))
        print('top1_acc:{}'.format(top1))
        print('top5_acc:{}'.format(top5))
    print('******final top1:{}'.format(top1_total / iterations))
    print('******final top5:{}'.format(top5_total / iterations))
    return top1_total / iterations


class AutoCalibrationEvaluator(AutoCalibrationEvaluatorBase):
    """auto calibration evaluator"""
    def __init__(self, target_loss, batch_num, args):
        """
            evaluate_batch_num is the needed batch num for evaluating
            the model. Larger evaluate_batch_num is recommended, because
            the evaluation metric of input model can be more precise
            with larger eval dataset.
        """
        self.target_loss = target_loss
        self.batch_num = batch_num
        self.args = args
        super().__init__()

    def calibration(self, model_file, weights_file):
        """"
        Function:
            do the calibration with model
        Parameter:
            model_file: the prototxt model define file of caffe model
            weights_file: the binary caffemodel file of caffe model
        """
        run_caffe_model(model_file, weights_file, self.batch_num)

    def evaluate(self, model_file, weights_file):
        """"
        Function:
            evaluate the model with batch_num of data, return the eval
            metric of the input model, such as top1 for classification
            model, mAP for detection model and so on.
        Parameter:
            model_file: the prototxt model define file of caffe model
            weights_file: the binary caffemodel file of caffe model
        """
        return do_benchmark_test(self.args, model_file, weights_file,
                                 self.args.iterations)

    def metric_eval(self, original_metric, new_metric):
        """
        Function:
            whether the metric of new fake quant model can satisfy the
            requirement
        Parameter:
            original_metric: the metric of non quantized model
            new_metric: the metric of new quantized model
        """
        # the loss of top1 acc need to be less than 0.2%
        loss = original_metric - new_metric
        if loss * 100 < self.target_loss:
            return True, loss
        return False, loss


def main(args):
    """main function"""
    if args.gpu_id is not None:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        amct.set_gpu_mode()
    else:
        caffe.set_mode_cpu()

    # User model files
    model_file = os.path.realpath(args.model_file)
    weights_file = os.path.realpath(args.weights_file)

    # Run pre model test
    if not args.pre_test:
        do_benchmark_test(args, model_file, weights_file, args.iterations)
        print('[AMCT][INFO]Run Mobilenet_v2 without quantize success!')
        return
    # step 1: create the quant config file
    config_json_file = './config.json'
    skip_layers = []
    batch_num = CALIBRATION_BATCH_NUM
    activation_offset = True
    amct.create_quant_config(config_json_file, model_file, weights_file,
                             skip_layers, batch_num, activation_offset)

    scale_offset_record_file = os.path.join(TMP, 'scale_offset_record.txt')
    result_path = os.path.join(RESULT, 'MobileNetV2')
    evaluator = AutoCalibrationEvaluator(target_loss=0.2, batch_num=batch_num,
                                         args=args)

    # step 2: start the accuracy_based_auto_calibration process
    amct.accuracy_based_auto_calibration(
        args.model_file,
        args.weights_file,
        evaluator,
        config_json_file,
        scale_offset_record_file,
        result_path)


if __name__ == '__main__':
    main(QUANT_ARGS)
