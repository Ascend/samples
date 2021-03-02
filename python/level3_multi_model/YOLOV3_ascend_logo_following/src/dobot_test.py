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
from serial.tools import list_ports
import pydobot
import time

# dobot config
port = list_ports.comports()[0].device
device = pydobot.Dobot(port=port, verbose=True)

# device.lock
(x, y, z, r, j1, j2, j3, j4) = device.pose()
print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

# test dobot
for i in range(10):
    device.move_to(x + 60, y + 60, z + 30, 0, wait=True)  # we wait until this movement is done before continuing
    device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing

    device.move_by_angle(j1, j2, j3 + 3, j4, wait=True)
    device.move_by_angle(j1, j2, j3, j4, wait=True)

device.close()
