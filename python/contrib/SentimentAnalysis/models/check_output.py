# coding=utf-8

# Copyright 2020 Huawei Technologies Co., Ltd
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
import sys
import os
import numpy as np
import random
import argparse
import pickle

from handle_data import dataLoader, CreatVocab
from driver.Config import Configurable
from handle_data.CreatVocab import *
from handle_data.batch_iter import *


def read_bin(config):
    try:
        file_name = config.decode_path.split('.')[0] + ".bin"
        feature_arr = np.fromfile(file_name, dtype=np.int32).reshape(
            config.sentence_max_length, config.batch_size)
    except IOError as except_err:
        print(except_err)
        return 1
    else:
        print(feature_arr)
        print(feature_arr.shape)
        return 0


def read_output_bin(config):
    try:
        file_name = "../output/" + config.decode_path + "_output_0" + ".bin"
        print(file_name)
        logits_arr = np.fromfile(file_name, dtype=np.float32).reshape(
            config.batch_size, -1)
    except IOError as except_err:
        print(except_err)
        return 1
    else:
        print(logits_arr)
        print(logits_arr.shape)
        return 0, logits_arr

if __name__ == "__main__":
    random.seed(233)
    np.random.seed(233)

    # vocab_file=os.path.join('chinese_L-12_H-768_A-12', 'vocab.txt')
    predecode_path = './data/test.txt'  # 解码之前的文件路径
    dev_data, sentence_length = dataLoader.decoder_sentence(predecode_path)

    parse = argparse.ArgumentParser()
    parse.add_argument('--config_file', type=str, default='default.ini')
    parse.add_argument('--thread', type=int, default=1)
    parse.add_argument('--use_cuda', action='store_true', default=False)

    parse.add_argument('-bert_config_file',
                       type=str,
                       default=os.path.join('chinese_L-12_H-768_A-12',
                                            'bert_config.json'))
    parse.add_argument('-vocab_file',
                       type=str,
                       default=os.path.join('chinese_L-12_H-768_A-12',
                                            'vocab.txt'),
                       help='bert_vocab')
    parse.add_argument(
        '-max_seq_length',
        type=int,
        default=202,
        help=
        'The maximum total input sequence length after WordPiece tokenization.'
    )
    parse.add_argument(
        '-warmup_proportion',
        type=float,
        default=0.1,
        help='Proportion of training to perform linear learning rate warmup for '
        'E.g., 0.1 = 10% of training.')
    parse.add_argument('-do_lower_case',
                       type=bool,
                       default=True,
                       help='Whether to lower case the input text.')

    args, extra_args = parse.parse_known_args()
    config_ = Configurable(args.config_file, extra_args)

    id1 = 0
    id2 = 0
    id3 = 0
    id3, logits_arr_ = read_output_bin(config_)
    predicts = np.argmax(logits_arr_, axis=1)  #shape = (batch_size,)
    print(predicts)
    print(predicts.shape)

    if id1 == 0 and id2 == 0 and id3 == 0:
        print('success!')
    else:
        print('faild!')
