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
import sys
import datetime
import numpy
import random
import argparse
import pickle
import tensorflow as tf
from tensorflow.python.platform import gfile

from handle_data import dataLoader, CreatVocab
# from handle_data.CreatVocab import *
from handle_data.CreatVocab import graph_util
from handle_data.batch_iter import create_batch_iter, pair_data_variable
from model.lstm import Lstm
from driver.Config import Configurable
from bert.pretrain import modeling, tokenization


def train(train_data, dev_data, vocab, config,
          bert_config_):
    src_vocab, tgt_vocab = vocab
    print('init model')
    model = None
    print('start training...')
    use_cuda = False
    if config.use_cuda:
        use_cuda = True
    # saver = tf.train.Saver()
    with tf.Session(config=tf.ConfigProto(
            log_device_placement=use_cuda)) as sess:
        if config.load_model:
            a = 0
        else:
            sess.run(tf.global_variables_initializer())

        # if config.decode:
        decode(model, sess, dev_data, vocab, config,
                bert_config_)
        print('decode successful!')
        return 0

        writer = tf.summary.FileWriter("logs1/", sess.graph)
        evaluate(model, -1, sess, dev_data, vocab, config)
        for i in range(config.epochs):
            step = 1
            train_batch_iter = create_batch_iter(train_data,
                                                 config.batch_size,
                                                 shuffle=True)
            for batch in train_batch_iter:
                feature, target, word_list = pair_data_variable(
                    batch, src_vocab, tgt_vocab, config)

                sess.run(model.train_op,
                         feed_dict={
                             model.w: feature,
                             model.gold: target
                         })

                if step % config.test_interval == 0:
                    loss, acc = sess.run([model.loss_op, model.accuracy],
                                         feed_dict={
                                             model.w: feature,
                                             model.gold: target
                                         })
                    accuracy = acc / len(target) * 100
                    time_str = datetime.datetime.now().isoformat()
                    print('epoch:{} step:{}|{} acc={:.2f}% loss={:.5f}'.format(
                        i, step, time_str, accuracy, loss))
                step += 1
            evaluate(model, i, sess, dev_data, vocab, config)


def decode(model, sess, dev_data, vocab, config):
    src_vocab, tgt_vocab = vocab
    # load model
    print('existed model path is :    ' + config.save_dirs + '/' +
          config.save_model_path)
    graph_def = tf.GraphDef()
    with open(config.save_dirs + '/' + config.save_model_path, 'rb') as f:
        graph_def.ParseFromString(f.read())
        sess.graph.as_default()
        tf.import_graph_def(graph_def, name='')

    sess = tf.Session()
    sess.run(tf.global_variables_initializer())

    _w = sess.graph.get_tensor_by_name('w:0')
    # _gold = sess.graph.get_tensor_by_name('gold:0')
    logits = sess.graph.get_tensor_by_name('s/logits:0')
    model.dropout = 0
    w = []
    pre = []
    print('start decode...')
    train_batch_iter = create_batch_iter(dev_data,
                                         config.batch_size,
                                         shuffle=True)
    print("config.decode_path:  ", config.decode_path)
    with open(config.decode_path, 'w', encoding='utf8') as f:
        for batch in train_batch_iter:
            feature, target, word_list = pair_data_variable(
                batch, src_vocab, tgt_vocab, config)

            logits = sess.run(
                logits,
                feed_dict={_w: feature
                           # _gold: target
                           })
            print("feature:  ", feature)
            print("---------------------------------------------------")
            print("target:   ", target)
            print("---------------------------------------------------")
            print("logits    ", logits)
            print("---------------------------------------------------")
            predicts = np.argmax(logits, axis=1).tolist()
            print("predicts    ", predicts)
            for id_, index in enumerate(predicts):
                pre.append(predicts[id_])
                w.append(word_list[id_])

        #save file
        # s_input = ''
        # for id, s_list in enumerate(w):
        #     if s_list == 0:
        #         continue
        #     for idx, l_list in enumerate(s_list):
        #         s_input += l_list
        #     f.write(s_input + ' ' + tgt_vocab.id2word(pre[id]) + '\n')
        #     s_input = ''


def evaluate(model, epoch, sess, dev_data, vocab, config):
    src_vocab, tgt_vocab = vocab
    print('start evaluate...')
    total_acc = 0
    gold_num = 0
    model.dropout = 0
    train_batch_iter = create_batch_iter(dev_data,
                                         config.batch_size,
                                         shuffle=True)
    for batch in train_batch_iter:
        feature, target, word_list = pair_data_variable(
            batch, src_vocab, tgt_vocab, config)

        gold_num += len(target)
        loss, acc = sess.run([model.loss_op, model.accuracy],
                             feed_dict={
                                 model.w: feature,
                                 model.gold: target
                             })
        total_acc += acc
    accuracy = total_acc / gold_num * 100
    _best_acc = best_acc
    print('acc={:.2f}%'.format(accuracy))
    if accuracy > _best_acc:
        _best_acc = accuracy
        _best_epoch = epoch
        print('##Update! best_acc={:.2f}% in epoch {}'.format(
            _best_acc, _best_epoch))
        output_graph_def = graph_util.convert_variables_to_constants(
            sess, sess.graph_def, output_node_names=['s/logits'])
        with tf.gfile.GFile(config.save_dirs + '/' + config.save_model_path,
                            mode='wb') as f:
            f.write(output_graph_def.SerializeToString())
        print('saved model successfully! in ' + config.save_dirs + '/' +
              config.save_model_path)
    else:
        print('not update, best_acc={:.2f}% in epoch {}'.format(
            _best_acc, _best_epoch))


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
    config_ = Configurable(args.config_file, extra_args)
    bert_config = modeling.BertConfig.from_json_file(args.bert_config_file)

    if args.max_seq_length > bert_config.max_position_embeddings:
        raise ValueError(
            "Cannot use sequence length %d because the BERT model "
            "was only trained up to sequence length %d" %
            (args.max_seq_length, bert_config.max_position_embeddings))

    tokenizer = tokenization.FullTokenizer(vocab_file=args.vocab_file,
                                           do_lower_case=args.do_lower_case)

    print("this is decode --------------")
    path = './data/test.txt'
    dev_data_, sentence_length = dataLoader.decoder_sentence(path)
    print(config_.save_dirs + '/' + config_.word_path)

    with open(config_.save_dirs + '/' + config_.word_path, 'rb') as f_:
        src_vocab_ = pickle.load(f_)
    with open(config_.save_dirs + '/' + config_.label_path, 'rb') as f_:
        tgt_vocab_ = pickle.load(f_)

    best_acc = 0
    best_epoch = 0
    train("", dev_data_, (src_vocab_, tgt_vocab_), tgt_vocab_.size, config_,
          bert_config)
