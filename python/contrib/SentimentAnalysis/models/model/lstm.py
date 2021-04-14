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
import tensorflow as tf
from tensorflow.contrib import rnn

from bert.pretrain import modeling


class Lstm(object):
    """
    LSTM model.
    """
    def __init__(self, label_size, config, bert_config):
        self.config = config
        self.w = tf.placeholder(
            dtype=tf.int32,
            shape=[config.sentence_max_length, config.batch_size],
            name='w')
        self.gold = tf.placeholder(dtype=tf.int32,
                                   shape=[config.batch_size, 3],
                                   name='gold')
        model = modeling.BertModel(config=bert_config,
                                   is_training=True,
                                   input_ids=self.w)
        init_checkpoint = "chinese_L-12_H-768_A-12/bert_model.ckpt"
        tvars = tf.trainable_variables()
        (assignment_map, initialized_variable_names
         ) = modeling.get_assignment_map_from_checkpoint(
             tvars, init_checkpoint)
        tf.train.init_from_checkpoint(init_checkpoint, assignment_map)

        self.embedding = model.get_sequence_output()

        with tf.name_scope("bilstm"):
            rnn_unit = bert_config.hidden_size
            basicLstm = tf.nn.rnn_cell.BasicLSTMCell(
                rnn_unit)  #  set rnn hidden parameters
            cell = tf.nn.rnn_cell.MultiRNNCell([
                basicLstm for i in range(config.num_layers)
            ])  # inpout multiple hidden layers
            x = tf.reshape(self.embedding, [-1, bert_config.hidden_size])
            x = tf.split(x, config.batch_size)
            outputs_tuple, _ = tf.nn.static_rnn(
                cell, x, dtype=tf.float32
            )  # us tensor interface to link cell into rnn network

            h = tf.transpose(outputs_tuple, perm=[0, 1, 2])
            h = tf.transpose(h, perm=[0, 2, 1])
            h = tf.tanh(h)
            h = tf.reduce_max(h, 2)
            h = tf.tanh(h)

        with tf.name_scope("s"):
            linear_weight = tf.Variable(tf.random_uniform(
                [bert_config.hidden_size, label_size]),
                                        trainable=True)
            bias = tf.Variable(0, dtype=tf.float32)
            h = tf.matmul(h, linear_weight) + bias
            self.logits = tf.nn.leaky_relu(h, name='logits')

        with tf.name_scope("loss"):
            self.loss_op = tf.reduce_mean(
                tf.nn.softmax_cross_entropy_with_logits(logits=self.logits,
                                                        labels=self.gold))
            optimizer = tf.train.AdamOptimizer(learning_rate=config.lr)
            self.train_op = optimizer.minimize(self.loss_op)

        with tf.name_scope("accuracy"):
            prediction = tf.nn.softmax(self.logits)
            correct_pred = tf.equal(tf.argmax(prediction, 1),
                                    tf.argmax(self.gold, 1))
            self.accuracy = tf.reduce_sum(tf.cast(correct_pred, tf.float32))
