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
import argparse

import torch
from amct_pytorch.tensor_decompose import auto_decomposition

from common.model import Net


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='Tensor decomposition sample')
    parser.add_argument('--pretrained-path', type=str, required=True,
                        help='Path to the pretrained weights')
    parser.add_argument('--decompose-info-path', type=str, required=True,
                        help='Decomposition information file path')
    parser.add_argument('--decomposed-weights-path', type=str, required=True,
                        help='Path to the decomposed weights')
    return parser.parse_args()


def main():
    """Main process."""
    args = parse_args()
    model = Net()
    model.load_state_dict(torch.load(args.pretrained_path, map_location='cpu'))
    model, _ = auto_decomposition(model, args.decompose_info_path)
    dec_weights_path = os.path.realpath(args.decomposed_weights_path)
    os.makedirs(os.path.dirname(dec_weights_path), exist_ok=True)
    torch.save(model.state_dict(), dec_weights_path)
    print('Decomposed weights file is saved to: {}'.format(
        dec_weights_path))


if __name__ == '__main__':
    main()
