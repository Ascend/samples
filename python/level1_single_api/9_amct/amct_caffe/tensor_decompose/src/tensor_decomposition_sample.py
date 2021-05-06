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

AMCT_CAFFE sample of tensor decomposition

"""

import argparse
import sys
import os


def add_path(path):
    """
    Function: Add path to env.
    """
    if path not in sys.path:
        sys.path.insert(0, path)


def parse_args():
    """
    Parse input arguments.
    """
    parser = argparse.ArgumentParser(description='tensor_decomposition demo')
    parser.add_argument('--model_file', type=str, required=True,
                        help='users caffe model file,prototxt for inference')
    parser.add_argument('--weights_file', type=str, required=True,
                        help='users caffe weight file')
    parser.add_argument('--new_model_file', type=str, required=True,
                        help='prototxt to save decomposed nets information.')
    parser.add_argument('--new_weights_file', type=str, required=True,
                        help='caffemodel to save all information')
    parser.add_argument('--caffe_dir', type=str, required=True,
                        help='Specify the dir of caffe')
    return parser.parse_args()


def main():
    """main process"""
    args = parse_args()

    add_path(os.path.join(args.caffe_dir, 'python'))
    from amct_caffe.tensor_decompose import \
        auto_decomposition # pylint: disable=E0401, C0415

    auto_decomposition(args.model_file, args.weights_file,
                       args.new_model_file, args.new_weights_file)


if __name__ == '__main__':
    main()
