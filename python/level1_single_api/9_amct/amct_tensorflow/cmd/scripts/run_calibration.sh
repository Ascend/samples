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

python3 ./src/process_data.py

amct_tensorflow calibration --model model/mobilenet_v2_1.0_224_frozen.pb  --outputs "MobilenetV2/Predictions/Reshape_1:0" --input_shape "input:32,224,224,3" --data_type "float32" --data_dir ./data/calibration --save_path output