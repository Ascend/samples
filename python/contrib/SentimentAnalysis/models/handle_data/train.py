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
sys.path.extend(['.', '..'])
import numpy as np
import datetime
import time
import tensorflow as tf
import tensorflow.contrib.rnn as rnn
from tensorflow.python.framework.graph_util import convert_variables_to_constants

from model.lstm import Lstm
from handle_data.batch_iter import pair_data_variable, create_batch_iter

best_acc = 0
best_epoch = 0


def train(train_data, dev_data, vocab, tgt_size, config,
          bert_config, tokenizer):
    """
    train api
    """
    src_vocab, tgt_vocab = vocab
    print('init model')
    model = Lstm(tgt_size, config, bert_config, tokenizer)
    model.dropout = config.dropout

    init = tf.global_variables_initializer()
    print('start training...')
    use_cuda = False
    if config.use_cuda:
        use_cuda = True
    saver = tf.train.Saver()
    with tf.Session(config=tf.ConfigProto(
            log_device_placement=use_cuda)) as sess:
        if config.load_model:  # false
            a = 0
        else:
            sess.run(init)

        if config.decode:  # false
            decode(model, sess, dev_data, vocab, config,
                   bert_config, tokenizer)
            print('decode successful!')
            return 0

        writer = tf.summary.FileWriter("logs1/", sess.graph)
        evaluate(model, -1, sess, dev_data, vocab, config)
        for i in range(config.epochs):
            print("begin the {}/{} of epochs.............".format(
                i, config.epochs))
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
    """
    decode data
    """
    src_vocab, tgt_vocab = vocab
    # load model
    graph_def = tf.GraphDef()
    with open(config.save_dirs + '/' + config.save_model_path, 'rb') as f:
        graph_def.ParseFromString(f.read())
        sess.graph.as_default()
        tf.import_graph_def(graph_def, name='')
    sess.run(tf.global_variables_initializer())
    _w = sess.graph.get_tensor_by_name('w:0')
    _gold = sess.graph.get_tensor_by_name('gold:0')
    logits = sess.graph.get_tensor_by_name('s/logits:0')

    model.dropout = 0
    w = []
    pre = []
    print('start decode...')
    train_batch_iter = create_batch_iter(dev_data,
                                         config.batch_size,
                                         shuffle=True)
    with open(config.decode_path, 'w', encoding='utf8') as f:
        for batch in train_batch_iter:
            feature, target, word_list = pair_data_variable(
                batch, src_vocab, tgt_vocab, config)

            logits = sess.run(logits, feed_dict={_w: feature, _gold: target})

            predicts = np.argmax(logits, axis=1).tolist()

            for id__, index in enumerate(predicts):
                pre.append(predicts[id__])
                w.append(word_list[id__])

        #save file
        s_input = ''
        for id__, s_list in enumerate(w):
            if s_list == 0:
                continue
            for idx, l_list in enumerate(s_list):
                s_input += l_list
            f.write(s_input + ' ' + tgt_vocab.id2word(pre[id__]) + '\n')
            s_input = ''


def evaluate(model, epoch, sess, dev_data, vocab, config):
    """
    evaluate model.
    """
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
    print('acc={:.2f}%'.format(accuracy))
    _best_acc = best_acc 
    _best_epoch = best_epoch
    if accuracy > _best_acc:
        _best_acc = accuracy
        _best_epoch = epoch
        print('##Update! best_acc={:.2f}% in epoch {}'.format(
            _best_acc, _best_epoch))
        output_graph_def = convert_variables_to_constants(
            sess, sess.graph_def, output_node_names=['s/logits'])
        with tf.gfile.GFile(config.save_dirs + '/' + config.save_model_path,
                            mode='wb') as f:
            f.write(output_graph_def.SerializeToString())
        print('saved model successfully! in ' + config.save_dirs + '/' +
              config.save_model_path)
    else:
        print('not update, best_acc={:.2f}% in epoch {}'.format(
            _best_acc, _best_epoch))
