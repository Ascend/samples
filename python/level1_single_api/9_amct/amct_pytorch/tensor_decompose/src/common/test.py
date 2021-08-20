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


import torch
import torch.nn.functional as F


def test(model, device, dataloader):
    """
    Test the model.

    Args:
        model: Model to test.
        device: Device to run the model.
        dataloader: Dataloader of test data.
    """
    model.eval()
    loss = 0
    correct = 0
    data_num = len(dataloader.dataset)
    with torch.no_grad():
        for data, target in dataloader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            loss += F.nll_loss(output, target, reduction='sum').item()
            prediction = output.argmax(dim=1, keepdim=True)
            correct += prediction.eq(target.view_as(prediction)).sum().item()
    loss /= data_num
    print('[Test] Loss: {:.6f}, Accuracy: {:.2f}% ({}/{})'.format(
        loss, 100 * correct / data_num, correct, data_num))
