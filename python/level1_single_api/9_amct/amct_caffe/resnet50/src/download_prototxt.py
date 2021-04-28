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

AMCT_CAFFE sample of ResNet50 model for download prototxt

"""
import os
import sys
import argparse
import urllib.request
from pathlib import Path

from google.protobuf import text_format

CUR_DIR = os.path.split(os.path.realpath(__file__))[0]
ROOT_DIR = os.path.join(CUR_DIR, '..')
RESNET50_DEPLOY_PROTOTXT_URL = 'https://raw.githubusercontent.com/KaimingHe/' \
    'deep-residual-networks/master/prototxt/ResNet-50-deploy.prototxt'
RESNET50_TRAIN_PROTOTXT_URL = 'https://raw.githubusercontent.com/antingshen/' \
    'resnet-protofiles/master/ResNet_50_train_val.prototxt'


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='ResNet50 demo')
    parser.add_argument('--caffe_dir',
                        dest='caffe_dir',
                        help='Specify the dir of caffe',
                        default=None,
                        type=str)
    parser.add_argument('--close_certificate_verify',
                        dest='close_certificate_verify',
                        help='close certificate verify',
                        action='store_true')
    return parser.parse_args()


def args_check_caffe_dir(args):
    """check args off caffe dir"""
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
args_check_caffe_dir(QUANT_ARGS)
add_path(os.path.join(QUANT_ARGS.caffe_dir, 'python'))
import caffe # pylint: disable=E0401, C0413
if QUANT_ARGS.close_certificate_verify:
    import ssl
    ssl._create_default_https_context = ssl._create_unverified_context # pylint: disable=W0212


def download_deploy_prototxt():
    """download ResNet50 caffe deploy model"""
    prototxt_path = os.path.join(ROOT_DIR, 'model',
                                 'ResNet-50-deploy.prototxt')
    if os.path.exists(prototxt_path):
        print("[INFO]'{}' already exist, no need to download.".format(
            prototxt_path))
        return
    urllib.request.urlretrieve(RESNET50_DEPLOY_PROTOTXT_URL, prototxt_path)

    net = caffe.proto.caffe_pb2.NetParameter()
    with open(prototxt_path, 'r') as file_open:
        text_format.Merge(file_open.read(), net)

    net.input.pop()
    input_dim_len = len(net.input_dim)
    for _ in range(input_dim_len):
        net.input_dim.pop()

    data_layer = caffe.proto.caffe_pb2.LayerParameter()
    data_layer.name = 'data'
    data_layer.type = 'Input'
    data_layer.top.append('data')
    data_layer.input_param.shape.append(caffe.proto.caffe_pb2.BlobShape())
    shape = data_layer.input_param.shape[0]
    shape.dim.MergeFrom([32, 3, 224, 224])
    net.layer.insert(0, data_layer)

    with open(prototxt_path, 'w') as file_open:
        file_open.write(text_format.MessageToString(net))
    print("[INFO]Download 'ResNet-50-deploy.prototxt' to '{}' success.".format(
        prototxt_path))


def download_train_prototxt():
    """download ResNet50 caffe train model"""
    prototxt_path = os.path.join(ROOT_DIR, 'model',
                                 'ResNet-50_retrain.prototxt')
    if os.path.exists(prototxt_path):
        print("[INFO]'{}' already exist, no need to download.".format(
            prototxt_path))
        return
    urllib.request.urlretrieve(RESNET50_TRAIN_PROTOTXT_URL, prototxt_path)

    net = caffe.proto.caffe_pb2.NetParameter()
    with open(prototxt_path, 'r') as file_open:
        text_format.Merge(file_open.read(), net)

    net.layer[1].data_param.batch_size = 32

    for i in net.layer:
        if i.HasField('batch_norm_param'):
            if i.batch_norm_param.HasField('use_global_stats'):
                i.batch_norm_param.ClearField('use_global_stats')

    with open(prototxt_path, 'w') as file_open:
        file_open.write(text_format.MessageToString(net))
    print(
        "[INFO]Download 'ResNet-50_retrain.prototxt' to '{}' success.".format(
            prototxt_path))


if __name__ == '__main__':
    download_deploy_prototxt()
    download_train_prototxt()
