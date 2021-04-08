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

AMCT_CAFFE sample of faster-rcnn model, init path env.
"""
import sys
import os


def add_path(path):
    """add path to sys"""
    if path not in sys.path:
        sys.path.insert(0, path)
# Add caffe to path
CAFFE_PATH = 'your_caffe_master_dir'
caffe_path = os.path.realpath(CAFFE_PATH)
# Add path of pycaffe
add_path(os.path.join(caffe_path, 'python'))

# Add lib to path
LIB_PATH = './python_tools'
lib_path = os.path.realpath(LIB_PATH)
add_path(lib_path)
