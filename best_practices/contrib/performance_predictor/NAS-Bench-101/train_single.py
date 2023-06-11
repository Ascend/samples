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


if __name__ == '__main__':
  tf.logging.set_verbosity(tf.logging.ERROR)
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_url',type=str,default="data/")
  parser.add_argument('--model_dir',type=str,default="checkpoint")
  args = parser.parse_args()
  models_file = args.data_url + "generated_graphs.json"
  with tf.gfile.Open(models_file) as f:
    models = json.load(f)
  model_id_regex="^"
  # regex = re.compile(model_id_regex)
  # evaluated_keys = [key for key in models.keys() if regex.match(key)]
  regex = re.compile(model_id_regex)
  evaluated_keys = [key for key in models.keys() if regex.match(key)]
  ordered_keys = sorted(evaluated_keys)
  num_models = len(ordered_keys)

  config = _config.build_config()
  config['train_seconds'] = -1      # Disable training time limit
  config['train_data_files'][0]=args.data_url +config['train_data_files'][0]
  config['train_data_files'][1]=args.data_url +config['train_data_files'][1]
  config['train_data_files'][2]=args.data_url +config['train_data_files'][2]
  config['train_data_files'][3]=args.data_url +config['train_data_files'][3]
  config['valid_data_file']=args.data_url + config['valid_data_file']
  config['test_data_file']=args.data_url + config['test_data_file']
  config['sample_data_file']=args.data_url + config['sample_data_file'] 
  # config['train_epochs']=5
  num = 0
  all_data = {}
  key = "09429ac82e3c35d01c30f78b4fd3d6d5"
  matrix, labels = models[key]
  matrix = np.array(matrix)
  available_ops = ['conv3x3-bn-relu', 'conv1x1-bn-relu', 'maxpool3x3']
  labels = (['input'] + [available_ops[lab] for lab in labels[1:-1]] + ['output'])
  spec = model_spec.ModelSpec(matrix, labels)
  data = evaluate.train_and_evaluate(spec, config, args.model_dir)
  print(data)

    
    
