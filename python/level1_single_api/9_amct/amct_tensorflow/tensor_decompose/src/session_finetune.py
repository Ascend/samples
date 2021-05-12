"""
# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""


import os
import argparse

from amct_tensorflow.tensor_decompose import decompose_graph

from common.tf_wrap import tf
from common.model import conv_net_example
from common.dataset import Mnist
from common.dataset import Dataset


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='tensor decomposition sample')
    parser.add_argument('--data_path', type=str, required=True,
                        help='path to data file')
    parser.add_argument('--save_path', type=str, required=True,
                        help='path to the saved decomposed model')
    parser.add_argument('--ckpt_path', type=str, required=True,
                        help='path to save result ckpt files')
    return parser.parse_args()


def main():
    """Main process"""
    args = parse_args()
    data_path = os.path.realpath(args.data_path)
    save_path = os.path.realpath(args.save_path)
    ckpt_path = os.path.realpath(args.ckpt_path)

    # Hyperparameters
    LEARNING_RATE = 0.001
    TOTAL_ITERS = 100
    BATCH_SIZE = 256

    # Prepare data
    (image_train, label_train), (image_val, label_val) = Mnist.load(
        data_path, one_hot=True)
    train_dataset = Dataset(image_train, label_train, BATCH_SIZE)

    # Build network
    input_image = tf.placeholder(
        'float', [None, Mnist.IMAGE_WIDTH, Mnist.IMAGE_HEIGHT, 1])
    input_label = tf.placeholder('float', [None, Mnist.CLASSES])
    logits = conv_net_example(input_image, Mnist.CLASSES)

    # Decompse graph and get restore variables,
    # after building model and before building optimizer
    decompose_graph(save_path)
    variables_to_restore = tf.global_variables()
    restorer = tf.train.Saver(variables_to_restore)

    # Loss and optimizer
    loss_op = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
        logits=logits, labels=input_label))
    optimizer = tf.train.AdamOptimizer(learning_rate=LEARNING_RATE)
    train_op = optimizer.minimize(
        loss_op, global_step=tf.train.get_global_step())
    correct = tf.equal(tf.argmax(logits, 1), tf.argmax(input_label, 1))
    accuracy = tf.reduce_mean(tf.cast(correct, tf.float32))

    variables_to_init = [
        v for v in tf.global_variables() if v not in variables_to_restore]
    init = tf.variables_initializer(variables_to_init)
    saver = tf.train.Saver()
    with tf.Session() as sess:
        sess.run(init)
        # Restore the decomposed ckpt files for finetuning
        restorer.restore(sess, args.save_path)

        # Train loop
        for step in range(1, TOTAL_ITERS + 1):
            batch_image, batch_label = train_dataset.next_batch()
            sess.run(train_op, feed_dict={
                input_image: batch_image, input_label: batch_label})
            if step == 1 or step % 20 == 0:
                loss, acc = sess.run([loss_op, accuracy], feed_dict={
                    input_image: batch_image, input_label: batch_label})
                print('Step: {}/{}, Train Loss: {:.4f}, Train Accuracy: {:.3f}'
                      .format(step, TOTAL_ITERS, loss, acc))
            if step == TOTAL_ITERS:
                os.makedirs(os.path.dirname(ckpt_path), exist_ok=True)
                saver.save(sess, ckpt_path, global_step=step)

        # Validation
        print('Validation Accuracy:', sess.run(accuracy, feed_dict={
            input_image: image_val, input_label: label_val}))

        # Convert to pb file
        graph_def = sess.graph.as_graph_def()
        output_graph_def = tf.graph_util.convert_variables_to_constants(
            sess, graph_def, ['dense/BiasAdd'])
        with tf.io.gfile.GFile(ckpt_path + '.pb', 'wb') as fid:
            fid.write(output_graph_def.SerializeToString())


if __name__ == '__main__':
    main()
