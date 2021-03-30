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

AMCT_MINDSPORE sample of ResNet50 model for download files

"""
import os
import stat
import argparse
import urllib.request


DIR_MODE = stat.S_IRWXU + stat.S_IRGRP + stat.S_IXGRP
CUR_DIR = os.path.split(os.path.realpath(__file__))[0]
SRC_DIR = "../src"

RESNET50_MODEL_DEFINE_URL = \
    'https://raw.githubusercontent.com/mindspore-ai/mindspore/v1.1.1/model_zoo/official/cv/resnet/src/resnet.py'
RESNET50_MODEL_DATASET_URL = \
    'https://raw.githubusercontent.com/mindspore-ai/mindspore/v1.1.1/model_zoo/official/cv/resnet/src/dataset.py'
RESNET50_LR_GENERATOR_URL = \
    'https://raw.githubusercontent.com/mindspore-ai/mindspore/v1.1.1/model_zoo/official/cv/resnet/src/lr_generator.py'
HCCL_TOOLS = \
    'https://raw.githubusercontent.com/mindspore-ai/mindspore/master/model_zoo/utils/hccl_tools/hccl_tools.py'


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='ResNet50 demo')

    parser.add_argument('--close_certificate_verify',
                        dest='close_certificate_verify',
                        help='close certificate verify',
                        action='store_true')
    return parser.parse_args()


ARGS = parse_args()

if ARGS.close_certificate_verify:
    import ssl
    ssl._create_default_https_context = ssl._create_unverified_context # pylint: disable=protected-access


def download_file(url_path, file_name):
    """ function for downloading files"""
    download_file_path = os.path.join(CUR_DIR, SRC_DIR, file_name)
    if os.path.exists(download_file_path):
        print("[INFO]'{}' already exist, no need to download.".format(download_file_path))
        return
    # check whether can access the file html page
    try:
        _ = urllib.request.urlopen(url_path, timeout=5).read()
    except Exception as exp: # pylint: disable=broad-except
        print('[ERROR] {}, Can not access {}, please check your network environment.'.format(exp, url_path))
        return

    urllib.request.urlretrieve(url_path, download_file_path)
    print("[INFO]Download file to '{}' success.".format(download_file_path))


if __name__ == '__main__':
    SRC_PATH = os.path.join(CUR_DIR, SRC_DIR)
    if not os.path.isdir(SRC_PATH):
        os.makedirs(SRC_PATH, DIR_MODE)
    download_file(RESNET50_MODEL_DEFINE_URL, 'resnet.py')
    download_file(RESNET50_MODEL_DATASET_URL, 'dataset.py')
    download_file(RESNET50_LR_GENERATOR_URL, 'lr_generator.py')
    download_file(HCCL_TOOLS, 'hccl_tools.py')
