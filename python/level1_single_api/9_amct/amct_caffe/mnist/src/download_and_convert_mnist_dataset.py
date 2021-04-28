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

Download mnist dataset and conver to lmdb format
"""
import os
from pathlib import Path
import wget # pylint: disable=E0401


def download_mnist_dataset(data_dir):
    """download mnist dataset"""
    if not Path(data_dir).exists():
        os.makedirs(data_dir)

    unzip_image_data = os.path.join(data_dir, 't10k-images-idx3-ubyte')
    if not Path(unzip_image_data).exists():
        image_data = os.path.join(data_dir, 't10k-images-idx3-ubyte.gz')
        if not Path(image_data).exists():
            print('[AMCT_CAFFE][INFO] Download test image data....')
            target_url = 'http://yann.lecun.com/exdb/mnist/{}.gz'.format('t10k-images-idx3-ubyte')
            wget.download(target_url, out=image_data)
            print('\n[AMCT_CAFFE][INFO] Download test image data done.')
        if not Path(image_data).exists():
            raise RuntimeError('Download t10k-images-idx3-ubyte failed')

        print('[AMCT_CAFFE][INFO] gunzip test image data...')
        os.system('cd {} && gunzip t10k-images-idx3-ubyte.gz -v && cd -'.format(data_dir))
        if not Path(unzip_image_data).exists():
            raise RuntimeError('Unzip data from {} failed'.format(image_data))

    unzip_label_data = os.path.join(data_dir, 't10k-labels-idx1-ubyte')
    if not Path(unzip_label_data).exists():
        label_data = os.path.join(data_dir, 't10k-labels-idx1-ubyte.gz')
        if not Path(label_data).exists():
            print('[AMCT_CAFFE][INFO] Download test label data....')
            target_url = 'http://yann.lecun.com/exdb/mnist/{}.gz'.format('t10k-labels-idx1-ubyte')
            wget.download(target_url, out=label_data)
            print('\n[AMCT_CAFFE][INFO] Download test label data done.')
        if not Path(label_data).exists():
            raise RuntimeError('Download t10k-labels-idx1-ubyte failed')

        print('[AMCT_CAFFE][INFO] gunzip test label data...')
        os.system('cd {} && gunzip t10k-labels-idx1-ubyte.gz -v && cd -'.format(data_dir))
        if not Path(unzip_label_data).exists():
            raise RuntimeError('Unzip data from {} failed'.format(label_data))


def make_mnist_lmdb_dataset(caffe_dir, data_dir, target_dir):
    """make mnist lmdb dataset"""
    if not Path(os.path.join(target_dir, 'mnist_test_lmdb')).exists():
        executor = os.path.join(caffe_dir, 'build/examples/mnist/convert_mnist_data.bin')
        if not Path(executor).exists():
            raise RuntimeError('Cannot find {}'.format(executor))

        data_dir = os.path.realpath(data_dir)
        target_dir = os.path.realpath(target_dir)
        print('[AMCT_CAFFE][INFO] convert mnist dataset to lmdb format')
        os.system('{0} {1}/t10k-images-idx3-ubyte {1}/t10k-labels-idx1-ubyte ' \
            '{2}/mnist_test_lmdb --backend=lmdb'.format(
                executor, data_dir, target_dir))
