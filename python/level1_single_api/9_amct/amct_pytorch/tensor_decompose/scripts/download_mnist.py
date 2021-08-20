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

from torchvision import datasets


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='download MNIST dataset')
    parser.add_argument('--data_path', type=str, required=True,
                        help='path to save MNIST data files')
    return parser.parse_args()


def main():
    """Main process"""
    args = parse_args()
    data_path = os.path.realpath(args.data_path)
    os.makedirs(os.path.dirname(data_path), exist_ok=True)
    print('Data will be downloaded to {}'.format(data_path))
    socket.setdefaulttimeout(60)  # Connection time limit for data downloading
    datasets.MNIST(data_path, download=True)
    print('Download completed')


if __name__ == '__main__':
    main()
