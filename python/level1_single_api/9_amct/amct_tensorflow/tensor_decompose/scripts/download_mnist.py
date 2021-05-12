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


import os
import socket
import argparse

import tensorflow.compat.v1 as tf  # make both TF v1 and v2 work


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='download MNIST dataset')
    parser.add_argument('--data_path', type=str, required=True,
                        help='path to save MNIST data file')
    return parser.parse_args()


def main():
    """Main process"""
    args = parse_args()
    data_path = os.path.realpath(args.data_path)
    if not data_path.endswith('mnist.npz'):
        data_path = os.path.join(data_path, 'mnist.npz')
    os.makedirs(os.path.dirname(data_path), exist_ok=True)
    print("Data will be downloaded to {}".format(data_path))
    socket.setdefaulttimeout(5)  # data download time limit
    (_, _), (_, _) = tf.keras.datasets.mnist.load_data(data_path)
    print("Download completed")


if __name__ == "__main__":
    main()
