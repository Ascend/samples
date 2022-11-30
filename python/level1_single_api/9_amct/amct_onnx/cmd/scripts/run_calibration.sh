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
#step1：将data/images目录下的jpg文件处理为data/calibration目录下的bin文件。
python3 ./src/process_data.py
#step2：进行模型量化操作。
amct_onnx calibration --model model/resnet101_v11.onnx --input_shape "input:16,3,224,224" --data_type "float32" --data_dir ./data/calibration/ --save_path ./results/resnet101_v11