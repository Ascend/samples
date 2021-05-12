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

from common.tf_wrap import tf
from common.model import conv_net_example
from common.dataset import Mnist


def parse_args():
    """
    Parse input arguments.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description='tensor decomposition sample')
    parser.add_argument('--data_path', type=str, required=True,
                        help='path to data file')
    parser.add_argument('--ckpt_path', type=str, required=True,
                        help='path to save result ckpt files')
    return parser.parse_args()


def model_fn(features, labels, mode, params):
    """
    Model function for estimator.

    Args:
        features: Input features (images).
        labels: Input labels.
        mode: Estimator mode.
        params: Parameters passed from estimator.

    Returns:
        EstimatorSpec for estimator.
    """
    logits = conv_net_example(features, Mnist.CLASSES)
    predict_classes = tf.argmax(logits, axis=1)

    if mode == tf.estimator.ModeKeys.PREDICT:
        return tf.estimator.EstimatorSpec(mode, predictions=predict_classes)

    loss_op = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(
        logits=logits, labels=tf.cast(labels, dtype=tf.int32)))
    optimizer = tf.train.AdamOptimizer(learning_rate=params['learning_rate'])
    train_op = optimizer.minimize(
        loss_op, global_step=tf.train.get_global_step())
    acc_op = tf.metrics.accuracy(labels=labels, predictions=predict_classes)
    logging_hook = tf.train.LoggingTensorHook(
        {'loss': loss_op}, every_n_iter=20)
    checkpoint_dir, checkpoint_basename = os.path.split(params['ckpt_path'])
    saver_hook = tf.estimator.CheckpointSaverHook(
        checkpoint_dir,
        save_steps=params['save_steps'],
        checkpoint_basename=checkpoint_basename)

    return tf.estimator.EstimatorSpec(
        mode=mode,
        predictions=predict_classes,
        loss=loss_op,
        train_op=train_op,
        eval_metric_ops={'accuracy': acc_op},
        training_hooks=[logging_hook, saver_hook])


def main():
    """Main process"""
    args = parse_args()
    data_path = os.path.realpath(args.data_path)
    ckpt_path = os.path.realpath(args.ckpt_path)
    tf.logging.set_verbosity(tf.logging.INFO)

    # Hyperparameters
    LEARNING_RATE = 0.01
    TOTAL_ITERS = 200
    BATCH_SIZE = 256

    # Prepare data
    (image_train, label_train), (image_val, label_val) = Mnist.load(
        data_path, one_hot=False)

    # Build estimator
    estimator = tf.estimator.Estimator(model_fn, params={
        'ckpt_path': ckpt_path,
        'save_steps': TOTAL_ITERS,
        'learning_rate': LEARNING_RATE})

    # Training
    input_fn = tf.estimator.inputs.numpy_input_fn(
        x=image_train,
        y=label_train,
        batch_size=BATCH_SIZE,
        num_epochs=None,
        shuffle=True)
    estimator.train(input_fn, steps=TOTAL_ITERS)
    final_ckpt_path = ckpt_path + '-{}'.format(TOTAL_ITERS)

    # Validation
    input_fn = tf.estimator.inputs.numpy_input_fn(
        x=image_val,
        y=label_val,
        batch_size=BATCH_SIZE,
        shuffle=False)
    res = estimator.evaluate(input_fn, checkpoint_path=final_ckpt_path)
    print('Validation Accuracy:', res['accuracy'])

    # Convert to pb file
    sess = tf.Session()
    with sess.as_default() as sess:
        saver = tf.train.import_meta_graph(final_ckpt_path + '.meta')
        sess.run(tf.global_variables_initializer())
        saver.restore(sess, final_ckpt_path)
    graph_def = sess.graph.as_graph_def()
    output_graph_def = tf.graph_util.convert_variables_to_constants(
        sess, graph_def, ['dense/BiasAdd'])
    with tf.io.gfile.GFile(final_ckpt_path + '.pb', 'wb') as fid:
        fid.write(output_graph_def.SerializeToString())


if __name__ == '__main__':
    main()
