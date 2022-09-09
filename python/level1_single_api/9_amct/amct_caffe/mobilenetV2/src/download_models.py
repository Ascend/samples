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

Model files downloading script for AMCT_CAFFE MoblieNetV2 sample.

"""
import os
import sys
import argparse
import urllib.request
from pathlib import Path
import stat
from google.protobuf import text_format # pylint: disable=E0401

DIR_MODE = stat.S_IRWXU + stat.S_IRGRP + stat.S_IXGRP
CUR_DIR = os.path.split(os.path.realpath(__file__))[0]
CUR_DIR = os.path.realpath(os.path.join(CUR_DIR, '..'))

MOBILENET_V2_MODEL_DEFINE_URL = 'https://raw.githubusercontent.com/shicai/' \
    'MobileNet-Caffe/master/mobilenet_v2_deploy.prototxt'
MOBILENET_V2_CAFFEMODEL_URL = 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/' \
    '003_Atc_Models/AE/ATC%20Model/mobilenetV2/mobilenet_v2.caffemodel'


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='MoblieNet V2 demo')
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
args_check_caffe_dir(QUANT_ARGS)
add_path(os.path.join(QUANT_ARGS.caffe_dir, 'python'))

import caffe # pylint: disable=E0401, C0413


if QUANT_ARGS.close_certificate_verify:
    import ssl
    ssl._create_default_https_context = ssl._create_unverified_context # pylint: disable=W0212


def download_file(url_path, file_name):
    """download file"""
    download_file_path = os.path.join(CUR_DIR, 'model', file_name)
    if os.path.exists(download_file_path):
        print("[INFO]'{}' already exist, no need to download.".format(
            download_file_path))
        return
    # check whether can access the file html page
    try:
        urllib.request.urlopen(url_path, timeout=5).read()
    except Exception as exception: # pylint: disable=W0703
        print('[ERROR] {}, Can not access {}, please check your network ' \
              'environment.'.format(exception, url_path))
        return

    urllib.request.urlretrieve(url_path, download_file_path)
    print("[INFO]Download file_name to '{}' success.".format(
        download_file_path))


def download_deploy_prototxt():
    """download MobileNetV2 caffe deploy prototxt"""
    prototxt_path = os.path.join(CUR_DIR, 'model',
                                 'mobilenet_v2_deploy.prototxt')
    if os.path.exists(prototxt_path):
        print("[INFO]'{}' already exist, no need to download.".format(
            prototxt_path))
        return
    urllib.request.urlretrieve(MOBILENET_V2_MODEL_DEFINE_URL, prototxt_path)

    net = caffe.proto.caffe_pb2.NetParameter()
    with open(prototxt_path, 'r') as prototxt_file:
        text_format.Merge(prototxt_file.read(), net)

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

    with open(prototxt_path, 'w') as prototxt_file:
        prototxt_file.write(text_format.MessageToString(net))
    print("[INFO]Download 'mobilenet_deploy.prototxt' to '{}' success.".format(
        prototxt_path))


def main():
    """ main function"""
    pre_model_path = os.path.join(CUR_DIR, 'model')
    if not os.path.isdir(pre_model_path):
        os.makedirs(pre_model_path, DIR_MODE)
    download_deploy_prototxt()
    download_file(MOBILENET_V2_CAFFEMODEL_URL, 'mobilenet_v2.caffemodel')


if __name__ == '__main__':
    main()
