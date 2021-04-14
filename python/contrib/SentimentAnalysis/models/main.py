"""coding=utf-8
Copyright 2020 Huawei Technologies Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
============================================================================
"""
import os

import numpy as np
import random
import argparse
import pickle
import tensorflow as tf

from driver.Config import Configurable
from handle_data import dataLoader, CreatVocab
# from handle_data.CreatVocab import *
from handle_data.train import train
from bert.pretrain import modeling, tokenization

if __name__ == '__main__':
    random.seed(233)
    np.random.seed(233)
    tf.set_random_seed(233)

    # parameters
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
    config = Configurable(args.config_file, extra_args)
    bert_config = modeling.BertConfig.from_json_file(args.bert_config_file)

    if args.max_seq_length > bert_config.max_position_embeddings:
        raise ValueError(
            "Cannot use sequence length %d because the BERT model "
            "was only trained up to sequence length %d" %
            (args.max_seq_length, bert_config.max_position_embeddings))

    tokenizer = tokenization.FullTokenizer(vocab_file=args.vocab_file,
                                           do_lower_case=args.do_lower_case)

    if config.decode:
        path = './data/test.txt'
        dev_data, sentence_length = dataLoader.decoder_sentence(path)
        with open(config.save_dirs + '/' + config.word_path, 'rb') as f:
            src_vocab = pickle.load(f)
        with open(config.save_dirs + '/' + config.label_path, 'rb') as f:
            tgt_vocab = pickle.load(f)
        train("", dev_data, (src_vocab, tgt_vocab), tgt_vocab.size, config,
              bert_config, tokenizer)
    else:
        train_data, res = dataLoader.read_sentence(
            "./data/train_hotel.txt", True)
        sentence_length, src_dic, tgt_dic = res
        dev_data, sentence_length = dataLoader.read_sentence(
            "./data/dev_hotel.txt", False)
        src_vocab, tgt_vocab = CreatVocab.create_vocabularies(
            train_data, 20000, src_dic, tgt_dic)
        print("src_vocab:", src_vocab.size)
        print("tgt_vocab:", tgt_vocab.size)
        with open(config.save_dirs + '/' + config.word_path, 'wb') as f:
            pickle.dump(src_vocab, f)
        print("save src_vocab successfully in " + config.save_dirs + '/' +
              config.word_path)
        with open(config.save_dirs + '/' + config.label_path, 'wb') as f:
            pickle.dump(tgt_vocab, f)
        print("save tgt_vocab successfully in " + config.save_dirs + '/' +
              config.label_path)
        train(train_data, dev_data, (src_vocab, tgt_vocab), tgt_vocab.size,
              config, bert_config, tokenizer)
