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
"""
Model Zoo API for Detectron2: a collection of functions to create common model architectures and
optionally load pre-trained weights as released in
`MODEL_ZOO.md <https://github.com/facebookresearch/detectron2/blob/master/MODEL_ZOO.md>`_.
"""
from .model_zoo import get, get_config_file, get_checkpoint_url

__all__ = ["get_checkpoint_url", "get", "get_config_file"]
