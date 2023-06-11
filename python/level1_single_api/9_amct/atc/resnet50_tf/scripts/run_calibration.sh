#!/bin/bash
# Copyright 2019 Huawei Technologies Co., Ltd
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
set -e

atc_app=atc
model_name=resnet_v1_50
model=./model/resnet_v1_50.pb
input_shape="input:16,224,224,3" # N is same with calibration_data
cfg_name=./config/compression_opt.config

python3.7 ./src/process_data.py

$atc_app --framework=3 --model=$model --input_shape=$input_shape \
--output=./outputs/$model_name \
--compression_optimize=$cfg_name --soc_version=Ascend310P3 --log=info
