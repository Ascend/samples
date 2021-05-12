"""
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""


import argparse
from amct_tensorflow.tensor_decompose import auto_decomposition


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='tensor decomposition sample')
    parser.add_argument('--meta_path', type=str, required=True,
                        help='input meta file path')
    parser.add_argument('--ckpt_path', type=str, required=True,
                        help='input ckpt file path')
    parser.add_argument('--save_path', type=str, required=True,
                        help='path to save decomposed model')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    # Please make sure the meta_path can be loaded by
    # tf.compat.v1.train.import_meta_graph. For example, if your model was
    # trained with horovod.tensorflow, then "import horovod.tensorflow" is
    # required before calling tf.compat.v1.train.import_meta_graph, therefore
    # you should import horovod.tensorflow before calling auto_decomposition.
    auto_decomposition(args.meta_path, args.ckpt_path, args.save_path)
