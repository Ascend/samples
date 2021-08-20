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


import torch.nn as nn
import torch.nn.functional as F


class Net(nn.Module):
    """Network definition."""

    def __init__(self, c_in=1, h_in=28, w_in=28, classes=10):
        """
        Network initialize method.

        Args:
            c_in: Input tensor channels.
            h_in: Input tensor height.
            w_in: Input tensor width.
            classes: Number of classes.
        """
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(c_in, 128, 3, 1)
        self.conv2 = nn.Conv2d(self.conv1.out_channels, 128, 3, 1)
        self.relu = nn.ReLU(inplace=True)
        self.flatten = nn.Flatten(1)
        self.fc = nn.Linear((h_in - 4) * (w_in - 4) * 128, classes)
        nn.init.kaiming_normal_(self.conv1.weight, mode='fan_out')
        nn.init.constant_(self.conv1.bias, 0)
        nn.init.kaiming_normal_(self.conv2.weight, mode='fan_out')
        nn.init.constant_(self.conv2.bias, 0)
        nn.init.normal_(self.fc.weight, 0, 0.01)
        nn.init.constant_(self.fc.bias, 0)

    def forward(self, x):
        """
        Network forward method.

        Args:
            x: Input tensor.

        Returns:
            Output tensor.
        """
        x = self.conv1(x)
        x = self.relu(x)
        x = self.conv2(x)
        x = self.relu(x)
        x = self.flatten(x)
        x = self.fc(x)
        x = F.log_softmax(x, dim=1)
        return x
