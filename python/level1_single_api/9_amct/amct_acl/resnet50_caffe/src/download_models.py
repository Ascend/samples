#!/usr/bin/env python3
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

AMCT_ACL sample of ResNet50 to download the .prototxt & .caffemodel files.

"""
import os
import argparse
import urllib.request


CUR_DIR = os.path.split(os.path.realpath(__file__))[0]
CUR_DIR = os.path.realpath(os.path.join(CUR_DIR, '..'))

RESNET50_DEPLOY_PROTOTXT_URL = 'https://raw.githubusercontent.com/KaimingHe/' \
    'deep-residual-networks/master/prototxt/ResNet-50-deploy.prototxt'
RESNET50_CAFFE_MODEL_URL = 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/' \
    '003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-model.caffemodel'

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='amct_acl ResNet50 demo')
    parser.add_argument('--close_certificate_verify',
                        dest='close_certificate_verify',
                        help='close certificate verify',
                        action='store_true')
    return parser.parse_args()

QUANT_ARGS = parse_args()
if QUANT_ARGS.close_certificate_verify:
    import ssl
    ssl._create_default_https_context = ssl._create_unverified_context # pylint: disable=W0212


def download_deploy_prototxt():
    """download ResNet50 caffe deploy model"""
    prototxt_path = os.path.join(CUR_DIR, 'model',
                                 'ResNet-50-deploy.prototxt')
    if os.path.exists(prototxt_path):
        print("[INFO]'{}' already exist, no need to download.".format(
            prototxt_path))
        return
    urllib.request.urlretrieve(RESNET50_DEPLOY_PROTOTXT_URL, prototxt_path)
    print("[INFO]Download 'ResNet-50-deploy.prototxt' to '{}' success.".format(
        prototxt_path))

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

if __name__ == '__main__':
    download_deploy_prototxt()
    download_file(RESNET50_CAFFE_MODEL_URL, 'ResNet-50-model.caffemodel')
