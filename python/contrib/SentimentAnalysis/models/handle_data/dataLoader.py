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
import re
import collections


def clean_str(string):
    """
    Tokenization/string cleaning for all datasets except for SST.
    Original taken from https://github.com/yoonkim/CNN_sentence/blob/master/process_data.py
    """
    string = string.lower()
    string = re.sub(r",", r",", string)
    string = re.sub(r"\'s", r" \'s", string)
    string = re.sub(r"\'ve", r" \'ve", string)
    string = re.sub(r"n\'t", r" n\'t", string)
    string = re.sub(r"\'re", r" \'re", string)
    string = re.sub(r"\'d", r" \'d", string)
    string = re.sub(r"\'ll", r" \'ll", string)

    string = re.sub(r"^-user-$", "<user>", string)
    string = re.sub(r"^-url-$", "<url>", string)
    string = re.sub(r"^-lqt-$", r"\'", string)
    string = re.sub(r"^-rqt-$", r"\'", string)
    # or
    # string = re.sub(r"^-lqt-$", "\"", string)
    # string = re.sub(r"^-rqt-$", "\"", string)
    string = re.sub(r"^-lrb-$", r"\(", string)
    string = re.sub(r"^-rrb-$", r"\)", string)
    string = re.sub(r"^lol$", "<lol>", string)
    string = re.sub(r"^<3$", "<heart>", string)
    string = re.sub(r"^#.*", "<hashtag>", string)
    string = re.sub(r"^[0-9]*$", "<number>", string)
    string = re.sub(r"^\:\)$", "<smile>", string)
    string = re.sub(r"^\;\)$", "<smile>", string)
    string = re.sub(r"^\:\-\)$", "<smile>", string)
    string = re.sub(r"^\;\-\)$", "<smile>", string)
    string = re.sub(r"^\;\'\)$", "<smile>", string)
    string = re.sub(r"^\(\:$", "<smile>", string)
    string = re.sub(r"^\)\:$", "<sadface>", string)
    string = re.sub(r"^\)\;$", "<sadface>", string)
    string = re.sub(r"^\:\($", "<sadface>", string)
    return string.strip()


def read_sentence(path, is_train):
    """
    read sentence.
    """
    data = []
    sentence_length = collections.Counter()
    feature_dict = collections.Counter()
    label_dict = collections.Counter()
    with open(path, "r", encoding="utf-8") as f:
        sentence = []
        for s in f:
            s = s.strip()
            s = s.split("|||")
            word = s[0]
            for j in word:
                sentence.append(j)
                if is_train:
                    feature_dict[j] += 1
            sentence_length[len(word)] += 1
            if is_train:
                label_dict[s[1]] += 1
            data.append((sentence, s[1]))
            sentence = []

    if is_train:
        res = [sentence_length, feature_dict, label_dict]
        return data, res

    return data, sentence_length


def decoder_sentence(path):
    """
    decode sentence.
    """
    data = []
    sentence_length = collections.Counter()
    with open(path, "r", encoding="utf-8") as f:
        sentence = []
        for s in f:
            s = s.strip()
            for j in s:
                sentence.append(j)
            sentence_length[len(s)] += 1
            data.append((sentence, '0'))
            sentence = []

    return data, sentence_length
