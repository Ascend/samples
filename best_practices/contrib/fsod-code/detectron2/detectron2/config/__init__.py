# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
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
from .compat import downgrade_config, upgrade_config
from .config import CfgNode, get_cfg, global_cfg, set_global_cfg, configurable

__all__ = [
    "CfgNode",
    "get_cfg",
    "global_cfg",
    "set_global_cfg",
    "downgrade_config",
    "upgrade_config",
    "configurable",
]
