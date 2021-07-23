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
import math
import argparse
import numpy as np
import tensorflow as tf # pylint: disable=E0401
from tqdm import trange # pylint: disable=E0401

import amct_tensorflow as amct # pylint: disable=E0401
from amct_tensorflow.common.auto_calibration import AutoCalibrationEvaluatorBase # pylint: disable=E0401


SIZE = 224
CENTRAL_FRACTION = 0.875
PATH = os.path.realpath('./')
OUTPUTS = os.path.join(PATH, 'outputs/accuracy_based_auto_calibration')
INPUTS = 'input'
PREDICTIONS = 'MobilenetV2/Predictions/Reshape_1'
EVAL_SIZE = 50000


def _decode(item):
    features = {
        'image/encoded': tf.io.FixedLenFeature((), tf.string, default_value=''),
        'image/format': tf.io.FixedLenFeature((), tf.string, default_value='jpeg'),
        'image/class/label': tf.io.FixedLenFeature([], dtype=tf.int64, default_value=-1),
        'image/class/text': tf.io.FixedLenFeature([], dtype=tf.string, default_value=''),
        'image/object/bbox/xmin': tf.io.VarLenFeature(dtype=tf.float32),
        'image/object/bbox/ymin': tf.io.VarLenFeature(dtype=tf.float32),
        'image/object/bbox/xmax': tf.io.VarLenFeature(dtype=tf.float32),
        'image/object/bbox/ymax': tf.io.VarLenFeature(dtype=tf.float32),
        'image/object/class/label': tf.io.VarLenFeature(dtype=tf.int64)}

    example = tf.io.parse_single_example(item, features)
    image = example['image/encoded']
    label = example['image/class/label']
    text = example['image/class/text']
    image = tf.cond(tf.image.is_jpeg(image),
                    lambda: tf.image.decode_jpeg(image),
                    lambda: tf.image.decode_png(image))

    return image, label, text


def _parse_eval(item):
    image, label, text = _decode(item)
    image = tf.cond(
        tf.equal(tf.shape(image)[-1], 1), true_fn=lambda: tf.image.grayscale_to_rgb(image), false_fn=lambda: image)
    if image.dtype != tf.float32:
        image = tf.image.convert_image_dtype(image, dtype=tf.float32)
    image = tf.image.central_crop(image, CENTRAL_FRACTION)
    image = tf.expand_dims(image, 0)
    image = tf.compat.v1.image.resize_bilinear(image, [SIZE, SIZE])
    image = tf.squeeze(image, [0])
    image = tf.subtract(image, 0.5)
    image = tf.multiply(image, 2.0)

    return image, label, text


class TFRecordDataset(): # pylint: disable=R0902, R0903
    """parser TFRecord dataset and preprocessing"""
    def __init__(self, path, keyword, num_parallel_reads=4, batch_size=32):

        self.filenames = os.listdir(path)
        self.path = []
        for i in self.filenames:
            if keyword is None or keyword in i:
                self.path.append(os.path.join(path, i))

        self.num_parallel_reads = num_parallel_reads
        self.dataset = tf.data.TFRecordDataset(
            self.path, num_parallel_reads=self.num_parallel_reads)

        self.dataset = self.dataset.map(_parse_eval)

        self.batch_size = batch_size
        self.dataset = self.dataset.batch(self.batch_size)

        self.iterator = tf.compat.v1.data.make_one_shot_iterator(self.dataset)
        self.images, self.labels, self.texts = self.iterator.get_next()


class MobileNetV2Evaluator(AutoCalibrationEvaluatorBase):
    """The evaluator for MobileNetV2"""
    def __init__(self, data_path, keyword, num_parallel_reads, batch_size):
        self.data_path = data_path
        self.keyword = keyword
        self.num_parallel_reads = num_parallel_reads
        self.batch_size = batch_size
        self.diff = 0.5

    def calibration(self, graph, outputs):
        """do the calibration"""
        with graph.as_default():
            dataset = TFRecordDataset(self.data_path, self.keyword, self.num_parallel_reads, self.batch_size)
            session = tf.compat.v1.Session(graph=graph)
            inputs = session.graph.get_tensor_by_name(INPUTS + ':0')
            predictions = session.graph.get_tensor_by_name(outputs[0] + ':0')
            session.run(tf.compat.v1.global_variables_initializer())
            images, _ = session.run([dataset.images, dataset.labels])
            session.run(predictions, feed_dict={inputs: images})
            session.close()

    def evaluate(self, graph, outputs): # pylint: disable=R0914
        """evaluate the input models, get the eval metric of model"""
        with graph.as_default():
            dataset = TFRecordDataset(self.data_path, self.keyword, self.num_parallel_reads, self.batch_size)
            session = tf.compat.v1.Session(graph=graph)
            inputs = session.graph.get_tensor_by_name(INPUTS + ':0')
            predictions = session.graph.get_tensor_by_name(outputs[0] + ':0')

            count_1 = 0
            for _ in trange(math.ceil(EVAL_SIZE / ARGS.batch_size), ncols=100):
                images, labels = session.run([dataset.images, dataset.labels])
                prediction = session.run(predictions, feed_dict={inputs: images})
                top_1 = np.argmax(prediction, axis=1)
                count_1 += np.sum(labels == top_1)
            acc_1 = count_1 / EVAL_SIZE * 100
            session.close()
        return acc_1

    def metric_eval(self, original_metric, new_metric):
        """whether the gap between new metric and original metric can
        satisfy the requirement
        """
        loss = original_metric - new_metric
        if loss < self.diff:
            return True, loss
        return False, loss


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='MobileNetV2 Automatic Calibration DEMO')

    parser.add_argument(
        '--dataset', dest='dataset', type=str, default=None,
        help='The path of ILSVRC-2012-CLS image classification dataset with "TFRecord" format for evaluation.')
    parser.add_argument(
        '--keyword', dest='keyword', type=str, default=None,
        help='One keyword to filter evaluation dataset filenames.')
    parser.add_argument(
        '--num_parallel_reads', dest='num_parallel_reads', type=int, default=4,
        help='The number of files to read in parallel.')
    parser.add_argument(
        '--batch_size', dest='batch_size', type=int, default=32, help='The number of samples in each batch.')
    parser.add_argument(
        '--model', dest='model', type=str, default='./model/mobilenet_v2_1.0_224_frozen.pb',
        help='The path of MobileNetV2 pb model for evaluation.')

    return parser.parse_args()


def args_check(args):
    """Check the arguments."""
    if args.dataset is None:
        raise RuntimeError('Must specify a evaluation dataset path!')
    args.dataset = os.path.realpath(args.dataset)
    if not os.access(args.dataset, os.F_OK):
        raise RuntimeError('Must specify a valid evaluation dataset path!')
    if args.model is None:
        raise RuntimeError('Must specify a evaluation model path!')
    args.model = os.path.realpath(args.model)
    if not os.access(args.model, os.F_OK):
        raise RuntimeError('Must specify a valid evaluation model path!')


def main():
    """main process"""
    args_check(ARGS)

    outputs = [PREDICTIONS]
    record_file = os.path.join(OUTPUTS, 'record.txt')

    config_file = os.path.join(OUTPUTS, 'config.json')
    with tf.io.gfile.GFile(ARGS.model, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    amct.create_quant_config(config_file, graph)

    save_dir = os.path.join(OUTPUTS, 'mobilenet_v2')

    evaluator = MobileNetV2Evaluator(ARGS.dataset, ARGS.keyword, ARGS.num_parallel_reads, ARGS.batch_size)

    amct.accuracy_based_auto_calibration(ARGS.model, outputs, record_file, config_file, save_dir, evaluator)


if __name__ == '__main__':
    ARGS = parse_args()
    main()
