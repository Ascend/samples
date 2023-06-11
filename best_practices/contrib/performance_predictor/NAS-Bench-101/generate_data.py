# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
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
# ============================================================================
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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import os
import re

from absl import app
from absl import flags
from matplotlib.style import available
from lib import config as _config
from lib import evaluate
# from lib import model_metrics_pb2
from lib import model_spec
import numpy as np
import tensorflow as tf
import random
import argparse


def generate_data(num, arch_name):
  models_file = "./data/generated_graphs.json"
  with tf.gfile.Open(models_file) as f:
    models = json.load(f)
  model_id_regex="^"
  # regex = re.compile(model_id_regex)
  # evaluated_keys = [key for key in models.keys() if regex.match(key)]
  model_id = 1
  regex = re.compile(model_id_regex)
  evaluated_keys = [key for key in models.keys() if regex.match(key)]
  ordered_keys = sorted(evaluated_keys)
  num_models = len(ordered_keys)
  print("the num of data:")
  print(num_models)
  pick_index = []
  while True:
    a = random.choice(ordered_keys)
    if a not in pick_index:
        pick_index.append(a)
    if len(pick_index) >= num:
        break
  buckets = {}
  j=0
  for i in range(0,num):
    buckets[pick_index[i]] = models[pick_index[i]]
  with tf.gfile.Open("./data/"+arch_name+".json", 'w') as f:
    json.dump(buckets, f, sort_keys=True)

  # for i in pick_index:
  #   buckets[i] = models[i]

  # with tf.gfile.Open("./data/train_arch.json", 'w') as f:
  #   json.dump(buckets, f, sort_keys=True)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--num',type=int,default=10)
  parser.add_argument('--arch_file',type=str,default="train_arch")
  args = parser.parse_args()
  generate_data(args.num, args.arch_file)
  
  
