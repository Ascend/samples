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


import torch.nn.functional as F


def train(model, device, dataloader, optimizer, steps, log_steps, print_flag):
    """
    Train the model.

    Args:
        model: Model to train.
        device: Device to run the model.
        dataloader: Dataloader of train data.
        optimizer: Optimizer for training.
        steps: Number of training iterations.
        log_steps: Iterations interval for logging.
        print_flag: Whether to print training log.
    """
    model.train()
    step_num = 0
    while True:
        for data, target in dataloader:
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()
            output = model(data)
            loss = F.nll_loss(output, target)
            loss.backward()
            optimizer.step()
            step_num += 1
            if print_flag and step_num % log_steps == 0:
                print('[Train] Step[{}/{}] Loss: {:.6f}'.format(
                    step_num, steps, loss.item()), flush=True)
            if step_num == steps:
                return
