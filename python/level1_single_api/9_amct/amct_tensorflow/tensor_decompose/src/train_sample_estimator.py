#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

AMCT_TENSORFLOW sample of tensor decomposition

"""

import socket
import argparse
import tensorflow as tf # pylint: disable=E0401
from tensorflow.examples.tutorials.mnist import input_data # pylint: disable=E0611, E0401


# HyperParameters
LEARNING_RATE = 0.01
TOTAL_ITERS = 200
BATCH_SIZE = 256
CKPT_PATH = 'model/checkpoints/model.ckpt-{}'.format(TOTAL_ITERS)


def parse_args():
    """
    Parse input arguments.
    """
    parser = argparse.ArgumentParser(description='tensor_decomposition demo')
    parser.add_argument('--data_path', type=str, required=True, help='path to data')
    return parser.parse_args()


def conv_net_example(x_dict):
    """
    Create the neural network
    """
    images = x_dict['images']
    images = tf.reshape(images, shape=[-1, 28, 28, 1])

    # conv layer
    conv1 = tf.layers.conv2d(images, 128, 3, activation=tf.nn.relu)
    conv2 = tf.layers.conv2d(conv1, 128, 3, activation=tf.nn.relu)

    fc1 = tf.contrib.layers.flatten(conv2)
    out = tf.layers.dense(fc1, 10)

    return out


def model_fn(features, labels, mode):
    """
    Define the model_fn
    """
    logits = conv_net_example(features)
    predict_classes = tf.argmax(logits, axis=1)

    if mode == tf.estimator.ModeKeys.PREDICT:
        return tf.estimator.EstimatorSpec(mode, predictions=predict_classes)

    loss_op = tf.reduce_mean(
        tf.nn.sparse_softmax_cross_entropy_with_logits(
            logits=logits, labels=tf.cast(labels, dtype=tf.int32)
        )
    )
    optimizer = tf.train.AdamOptimizer(learning_rate=LEARNING_RATE)
    train_op = optimizer.minimize(loss_op, global_step=tf.train.get_global_step())
    acc_op = tf.metrics.accuracy(labels=labels, predictions=predict_classes)
    logging_hook = tf.train.LoggingTensorHook({"loss": loss_op},
                                              every_n_iter=20)
    saver_hook = tf.estimator.CheckpointSaverHook('model/checkpoints',
                                                  save_steps=TOTAL_ITERS)

    return tf.estimator.EstimatorSpec(mode=mode,
                                      predictions=predict_classes,
                                      loss=loss_op, train_op=train_op,
                                      eval_metric_ops={'accuracy': acc_op},
                                      training_hooks=[logging_hook, saver_hook])


def main():
    """main process"""
    args = parse_args()
    tf.logging.set_verbosity(tf.logging.INFO)

    # set time out threshold
    socket.setdefaulttimeout(5)
    # Dataset http://yann.lecun.com/exdb/mnist/
    mnist = input_data.read_data_sets(args.data_path, one_hot=False)

    # Build the Estimator, load decomposed_ckpt for finetune
    estimator = tf.estimator.Estimator(model_fn)

    # Training
    input_fn = tf.estimator.inputs.numpy_input_fn(x={'images': mnist.train.images},
                                                  y=mnist.train.labels,
                                                  batch_size=BATCH_SIZE,
                                                  num_epochs=None,
                                                  shuffle=True)
    estimator.train(input_fn, steps=TOTAL_ITERS)

    # Evaluating
    input_fn = tf.estimator.inputs.numpy_input_fn(x={'images': mnist.test.images},
                                                  y=mnist.test.labels,
                                                  batch_size=BATCH_SIZE,
                                                  shuffle=False)
    res = estimator.evaluate(input_fn, checkpoint_path=CKPT_PATH)
    print("Valid Accuracy:", res['accuracy'])


if __name__ == '__main__':
    main()
