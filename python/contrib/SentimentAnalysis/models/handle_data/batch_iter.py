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
import numpy as np
import random
random.seed(233)


def create_batch_iter(data, batch_size, shuffle=True):
    """
    create batch iter.
    """
    if shuffle:
        np.random.shuffle(data)

    # ordering
    # src_ids = sorted(range(data_size), key=lambda src_id: len(data[src_id][0]), reverse=True)
    # data = [data[src_id] for src_id in src_ids]

    batched_data = []
    instances = []
    for instance in data:
        instances.append(instance)
        if len(instances) == batch_size:
            batched_data.append(instances)
            instances = []

    if len(instances) > 0:
        batched_data.append(instances)

    for batch in batched_data:
        yield batch


def pair_data_variable(batch, vocab_srcs, vocab_tgts, config):
    """
    make pair of data variables.
    """
    batch_size = config.batch_size

    src_lengths = [len(batch[i][0]) for i in range(len(batch))]
    # in decreasing order
    if len(src_lengths) != config.batch_size:
        update_length = config.batch_size - len(src_lengths)
        for i in range(update_length):
            src_lengths.append(
                0)  # fix padding [4,3,..,0,0]  batchsize 

    max_src_length = config.sentence_max_length  #max(src_lengths)

    src_words = np.zeros([max_src_length, batch_size])
    tgt_words = []
    gold = np.zeros([batch_size, 3])

    # if config.use_cuda:
    #     src_words = src_words.cuda()
    #     tgt_words = tgt_words.cuda()

    word_list = []
    for idx, instance in enumerate(batch):
        # print("instance[0]=  {}".format(instance[0][0].encode('ascii', 'ignore')))
        # print("instance[1]=  ",instance[1][0])

        sentence = vocab_srcs.word2id(instance[0])
        word_list.append(instance[0])
        for idj, value in enumerate(sentence):
            src_words[idj][idx] = value
        tgt_words.append(vocab_tgts.word2id(instance[1]))

    if len(batch) != config.batch_size:  # fix padding 0
        for idx in range(len(batch), config.batch_size):
            word_list.append(0)
            tgt_words.append(0)

    for i, j in enumerate(tgt_words):
        if j == 0:
            gold[i][0] = 1
            gold[i][1] = 0
            gold[i][2] = 0
        elif j == 1:
            gold[i][0] = 0
            gold[i][1] = 1
            gold[i][2] = 0
        else:
            gold[i][0] = 0
            gold[i][1] = 0
            gold[i][2] = 1

    return src_words, gold, word_list
