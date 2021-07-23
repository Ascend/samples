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

AMCT_CAFFE sample of resnet-50 model

"""
import os
import sys
import argparse
from pathlib import Path
import numpy as np
import cv2 # pylint: disable=E0401

MODEL_INPUT_BLOB_NAME = 'data'
MODEL_OUTPUT_BLOB_NAME = 'prob'

PATH = os.path.split(os.path.realpath(__file__))[0]
PATH = os.path.realpath(os.path.join(PATH, '..'))
RESULT = os.path.join(PATH, 'results/convert_results')
DATA_DIR = os.path.join(PATH, 'data/images')
LABEL_FILE = os.path.join(DATA_DIR, 'image_label.txt')
BATCH_SIZE = 32

# Configuration for ImageNet LMDB dataset
SCALE = 1
CROP_SIZE = 224
MEAN_FILE = None
MEAN_VALUE = [103.894, 116.555, 122.578]


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='ResNet50 demo')
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode',
                        action='store_true')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use',
                        default=None, type=int)
    parser.add_argument('--model_file', dest='model_file',
                        help='Specify the model file of caffe model.',
                        default=None, type=str)
    parser.add_argument('--weights_file', dest='weights_file',
                        help='Specify the weights file of caffe model.',
                        default=None, type=str)
    parser.add_argument('--caffe_dir', dest='caffe_dir',
                        help='Specify the dir of caffe',
                        default=None, type=str)
    parser.add_argument('--pre_test', dest='pre_test',
                        help='Do test with amct caffe calibration or not',
                        default=False, action='store_true')
    parser.add_argument('--record_file', dest='record_file',
                        help='Specify scale offset record file',
                        default=None, type=str)
    parser.add_argument('--benchmark',
                        dest='benchmark',
                        help='Do benchmark test.',
                        action='store_true')
    parser.add_argument('--iterations',
                        dest='iterations',
                        help='Specify iterations of test.',
                        default=5,
                        type=int)
    parser.add_argument('--dataset',
                        dest='dataset',
                        help='The path of benchmark dataset.',
                        default=None,
                        type=str)
    return parser.parse_args()


def args_check(args):
    """Check resnet-50 sample args"""
    # --weights_file
    if args.weights_file is None:
        raise RuntimeError('Must specify a caffe caffemodel file')
    resnet_50_weights_file = os.path.realpath(args.weights_file)
    if not Path(resnet_50_weights_file).exists():
        raise RuntimeError('Must specify a caffe caffemodel file')
    args.weights_file = resnet_50_weights_file
    # --model_file
    if args.model_file is None:
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    resnet_50_model_file = os.path.realpath(args.model_file)
    if not Path(resnet_50_model_file).exists():
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    args.model_file = resnet_50_model_file
    # --caffe_dir
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
    # --iterations
    if not args.benchmark and args.iterations > 5:
        raise RuntimeError('Max iterations of sample dataset is 5')
    # --record_file
    record_file = os.path.realpath(args.record_file)
    if record_file is None:
        raise RuntimeError('Must specify a scale offset record file.')
    if not Path(record_file).exists():
        raise RuntimeError('Scale offset record file {} not exists.'\
            ''.format(record_file))
    if args.cpu_mode and args.gpu_id is not None:
        raise RuntimeError('Cannot run in CPU mode and GPU mode at same time.')
    args.record_file = record_file


def add_path(path):
    """Add path to env"""
    if path not in sys.path:
        sys.path.insert(0, path)


QUANT_ARGS = parse_args()
args_check(QUANT_ARGS)
add_path(os.path.join(QUANT_ARGS.caffe_dir, 'python'))


import caffe # pylint: disable=E0401, C0413
import amct_caffe as amct # pylint: disable=E0401, C0413


if QUANT_ARGS.benchmark:
    if QUANT_ARGS.dataset is None:
        raise RuntimeError(
            'Must specify the ImageNet path using in benchmark.')
    QUANT_ARGS.dataset = os.path.realpath(QUANT_ARGS.dataset)
    if not os.access(QUANT_ARGS.dataset, os.F_OK):
        raise RuntimeError('Must specify a valid path.')
    import datasets

def mkdir(name):
    """make dir"""
    if not os.access(name, os.F_OK):
        os.makedirs(name)


def get_blobs_from_im(data_dir, imgs, batch_size):
    """Read image files to blobs [3, 256, 256]"""
    if batch_size != len(imgs):
        raise RuntimeError(
            'batch_size:{} != len(imgs):{}'.format(batch_size, len(imgs)))

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
    return crop_data


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


def get_labels_from_txt():
    """Read all images' name and label from label_file"""
    images = []
    labels = []
    with open(LABEL_FILE, 'r') as file_open:
        lines = file_open.readlines()
        for line in lines:
            images.append(line.split(' ')[0])
            labels.append(int(line.split(' ')[1]))
    return images, labels


def run_caffe_model(model_file, weights_file, iterations):
    """run caffe model forward"""
    net = caffe.Net(model_file, weights_file, caffe.TEST)

    top1_total = 0
    top5_total = 0
    images, labels = get_labels_from_txt()
    for iter_num in range(iterations):
        blobs_data = get_blobs_from_im(
            DATA_DIR,
            images[iter_num * BATCH_SIZE: (iter_num + 1) * BATCH_SIZE],
            BATCH_SIZE)
        blobs_data = img_preprocess(blobs_data, [104, 117, 123], 224)
        forward_kwargs = {MODEL_INPUT_BLOB_NAME: blobs_data}
        blobs_out = net.forward(**forward_kwargs)
        top1, top5 = img_postprocess(blobs_out[MODEL_OUTPUT_BLOB_NAME], \
            labels[iter_num * BATCH_SIZE: (iter_num + 1) * BATCH_SIZE])
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
    return top1_total / iterations, top5_total / iterations


def main(args):
    """Main function"""
    args_check(args)
    mkdir(RESULT)

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
    if args.pre_test:
        if not args.benchmark:
            run_caffe_model(
                args.model_file, args.weights_file, args.iterations)
        else:
            do_benchmark_test(
                args, args.model_file, args.weights_file, args.iterations)
        print('[AMCT][INFO]Run ResNet-50 without quantize success!')
        return

    result_path = os.path.join(RESULT, 'ResNet50')
    amct.convert_model(args.model_file,
                       args.weights_file,
                       args.record_file,
                       result_path)

    # Phase4: do final fake quant model test
    fake_quant_model = os.path.join(
        RESULT, 'ResNet50_fake_quant_model.prototxt')
    fake_quant_weights = os.path.join(
        RESULT, 'ResNet50_fake_quant_weights.caffemodel')
    if not args.benchmark:
        run_caffe_model(
            fake_quant_model, fake_quant_weights, args.iterations)
    else:
        do_benchmark_test(
            args, fake_quant_model, fake_quant_weights, args.iterations)
    print('[AMCT][INFO]Run ResNet-50 with quantize success!')


if __name__ == '__main__':
    main(QUANT_ARGS)
