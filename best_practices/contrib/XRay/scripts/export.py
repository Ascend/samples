# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================


import os
import sys
import argparse
import numpy as np

cur_path = os.path.abspath(os.path.dirname(__file__))
root_path = os.path.split(cur_path)[0]
sys.path.append(root_path)

from core.models.unet import UNet

from mindspore import Tensor, export, load_checkpoint, load_param_into_net, context

parser = argparse.ArgumentParser(description="TernaryBert export model")
parser.add_argument("--device_target", type=str, default="Ascend", choices=["Ascend", "GPU"],
                    help="device where the code will be implemented. (Default: Ascend)")
parser.add_argument("--file_name", type=str, default="rilian", help="The name of the output file.")
parser.add_argument("--num_class", type=int, default=4, help="num class")
parser.add_argument("--file_format", type=str, default="MINDIR", choices=["AIR", "MINDIR"],
                    help="output model type")
parser.add_argument("--ckpt_file", type=str, default="./checkpoints/best_model_miou.ckpt", \
    help="pretrained checkpoint file")
parser.add_argument('--crop_size', type=int, default=512, help='crop image size')  # 480
args = parser.parse_args()

context.set_context(mode=context.GRAPH_MODE, device_target=args.device_target)
if args.device_target == "Ascend":
    context.set_context(device_id=0)

if __name__ == "__main__":
    net = UNet(3, args.num_class, bilinear=True).npu()
    param_dict = load_checkpoint(args.ckpt_file)
    load_param_into_net(net, param_dict)

    with torch.no_grad():
        input_data = Tensor(np.zeros((args.crop_size, args.crop_size), np.float32))
        export(net, *input_data, file_name=args.file_name, file_format=args.file_format)