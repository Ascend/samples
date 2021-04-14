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

UNK = 1
PAD = 0
PAD_S, UNK_S = '<pad>', '<unk>'


def create_vocabularies(vocab_size, src_word_counter, tgt_word_counter):
    """
    create vocabularies, from high to low frequency.
    """
    src_most_common = [
        ite for ite, it in src_word_counter.most_common(vocab_size)
    ]
    tgt_most_common = [
        ite for ite, it in tgt_word_counter.most_common(vocab_size)
    ]

    src_vocab = VocabSrc(src_most_common)
    tgt_vocab = VocabTgt(tgt_most_common)
    print(tgt_vocab)

    return src_vocab, tgt_vocab


class VocabSrc(object):
    """
    vocab of src .
    """
    def __init__(self, word_list):
        self._id2extword = [PAD_S, UNK_S]

        self.i2w = [PAD_S, UNK_S] + word_list
        self.w2i = {}
        for idx, word in enumerate(self.i2w):
            self.w2i[word] = idx

        if len(self.w2i) != len(self.i2w):
            print("serious bug: words dumplicated, please check!")
            self.copydict()

    def copydict(self):
        """
        copy dict.
        """
        w2i = self.i2w
        return w2i

    def word2id(self, xx):
        """
        convert word to id.
        """
        if isinstance(xx, list):
            return [self.w2i.get(word, UNK) for word in xx]
        return self.w2i.get(xx, UNK)

    def id2word(self, xx):
        """
        convert id to word.
        """
        if isinstance(xx, list):
            return [self.i2w[idx] for idx in xx]
        return self.i2w[xx]

    @property
    def size(self):
        """
        szie of dict
        """
        return len(self.i2w)

    @property
    def embed_size(self):
        """
        get embed size.
        """
        return len(self._id2extword)


class VocabTgt(object):
    """
    vocab to gt
    """
    def __init__(self, word_list):
        self.i2w = word_list
        self.w2i = {}
        for idx, word in enumerate(self.i2w):
            self.w2i[word] = idx
        if len(self.w2i) != len(self.i2w):
            print("serious bug: words dumplicated, please check!")

    def word2id(self, xx):
        """
        word to id.
        """
        if isinstance(xx, list):
            return [self.w2i[word] for word in xx]
        return self.w2i[xx]

    def id2word(self, xx):
        """
        convert id to word.
        """
        if isinstance(xx, list):
            return [self.i2w[idx] for idx in xx]
        return self.i2w[xx]

    @property
    def size(self):
        """
        size of dict
        """
        return len(self.i2w)
