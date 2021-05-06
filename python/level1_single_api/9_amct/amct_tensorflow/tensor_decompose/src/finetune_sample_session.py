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
from amct_tensorflow.tensor_decompose import decompose_graph # pylint: disable=E0401


# HyperParameters
LEARNING_RATE = 0.001
TOTAL_ITERS = 100
BATCH_SIZE = 256
NUM_INPUT = 784  # 28*28


def parse_args():
    """
    Parse input arguments.
    """
    parser = argparse.ArgumentParser(description='tensor_decomposition demo')
    parser.add_argument('--data_path', type=str, required=True, help='path to data')
    parser.add_argument('--save_path', type=str, required=True, help='path to saved decomposed model')
    return parser.parse_args()


def conv_net_example(features):
    """
    Create the neural network
    """
    features = tf.reshape(features, shape=[-1, 28, 28, 1])

    # conv layer
    conv1 = tf.layers.conv2d(features, 128, 3, activation=tf.nn.relu)
    conv2 = tf.layers.conv2d(conv1, 128, 3, activation=tf.nn.relu)

    fc1 = tf.contrib.layers.flatten(conv2)
    # MNIST has 10 classes
    out = tf.layers.dense(fc1, 10)

    return out


def main(): # pylint: disable=R0914
    """main process"""
    args = parse_args()

    # set time out threshold
    socket.setdefaulttimeout(5)
    # Dataset http://yann.lecun.com/exdb/mnist/
    mnist = input_data.read_data_sets(args.data_path, one_hot=True)

    input_x = tf.placeholder("float", [None, NUM_INPUT])
    input_y = tf.placeholder("float", [None, 10])
    logits = conv_net_example(input_x)

    # Decompse graph and get restore variables after building model and before building optimizer
    decompose_graph(args.save_path)
    variable_to_restore = tf.global_variables()
    restorer = tf.train.Saver(variable_to_restore)

    # loss and optimizer
    loss_op = tf.reduce_mean(
        tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=input_y)
    )
    optimizer = tf.train.AdamOptimizer(learning_rate=LEARNING_RATE)
    train_op = optimizer.minimize(loss_op, global_step=tf.train.get_global_step())

    correct = tf.equal(tf.argmax(logits, 1), tf.argmax(input_y, 1))
    accuracy = tf.reduce_mean(tf.cast(correct, tf.float32))

    variable_to_init = [val for val in tf.global_variables() if val not in variable_to_restore]
    init = tf.variables_initializer(variable_to_init)

    saver = tf.train.Saver()

    with tf.Session() as sess:
        sess.run(init)
        # restorer decomposed_ckpt for finetune
        restorer.restore(sess, args.save_path)

        # Train loop
        for step in range(1, TOTAL_ITERS + 1):
            batch_x, batch_y = mnist.train.next_batch(BATCH_SIZE)
            sess.run(train_op, feed_dict={input_x: batch_x, input_y: batch_y})
            if step == 1 or step % 20 == 0:
                loss, acc = sess.run([loss_op, accuracy], feed_dict={input_x: batch_x, input_y: batch_y})
                print("Step: {}/{}, Train Loss: {:.4f}, Train Accuracy: {:.3f}"
                      .format(step, TOTAL_ITERS, loss, acc))

            if step == TOTAL_ITERS:
                saver.save(sess, 'model/finetuned_ckpt/model.ckpt', global_step=step)

        print("Valid Accuracy:",
              sess.run(accuracy, feed_dict={input_x: mnist.test.images, input_y: mnist.test.labels}))

        # convert to pb
        graph_def = sess.graph.as_graph_def()
        output_graph_def = tf.compat.v1.graph_util.convert_variables_to_constants(sess,
                                                                                  graph_def,
                                                                                  ['dense/BiasAdd'])
        with tf.io.gfile.GFile('model/finetuned_ckpt/model.pb', 'wb') as fid:
            fid.write(output_graph_def.SerializeToString())


if __name__ == '__main__':
    main()
