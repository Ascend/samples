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

AMCT_TENSORFLOW sample of tensor decomposition

"""

import argparse
from amct_tensorflow.tensor_decompose import auto_decomposition # pylint: disable=E0401

if __name__ == '__main__':
    PARSER = argparse.ArgumentParser(description='tensor_decomposition demo')

    PARSER.add_argument('--meta_path', type=str, required=True,
                        help='users meta file path')
    PARSER.add_argument('--ckpt_path', type=str, required=True,
                        help='users weight file path')
    PARSER.add_argument('--save_path', type=str, required=True,
                        help='path to save decomposed model')
    ARGS = PARSER.parse_args()

    # Make sure your meta_path can load by 'tf.compat.v1.train.import_meta_graph'.
    # For example, if you train model ues horovod.tensorflow,
    # you must import horovod.tensorflow before calling import_meta_graph,
    # so you should import horovod.tensorflow before calling auto_decomposition.
    auto_decomposition(ARGS.meta_path, ARGS.ckpt_path, ARGS.save_path)
