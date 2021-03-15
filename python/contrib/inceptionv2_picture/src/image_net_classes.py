"""
# Copyright 2020 Huawei Technologies Co., Ltd
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

image_net_classes = [
"indoor after decoration parallel",
"indoor after decoration unparallel",
"outdoor after decoration parallel",
"outdoor after decoration unparalleled",
"outdoor before decoration parallel",
"outdoor before decoration unparallel"
]

def get_image_net_class(identifier):
    """
    get_image_net_class
    """
    if identifier >= len(image_net_classes):
        return "unknown"
    else:
        return image_net_classes[identifier]