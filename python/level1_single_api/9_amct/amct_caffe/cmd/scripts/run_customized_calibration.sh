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

model_name=ResNet-50-model
model=./model/ResNet-50-deploy.prototxt
weight=./model/ResNet-50-model.caffemodel
input_shape="data:1,3,224,224"
data_dir="data/image"

python3.7 ./src/process_data.py
amct_caffe calibration --model $model  --weights $weight --save_path ./results/resnet50_ --evaluator ./src/evaluator.py