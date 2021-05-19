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
import numpy as np
from PIL import Image # pylint: disable=E0401
import tensorflow as tf # pylint: disable=C0411, E0401

# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct # pylint: disable=E0401
amct.set_logging_level(print_level='info', save_level='info')


SIDE = 224
PATH = os.path.realpath('./')
OUTPUTS = os.path.join(PATH, 'outputs/nuq')


def load_image(path):
    """Load image to memory"""
    imagelist = []

    for file_name in os.listdir(path):
        image_path = os.path.join(path, file_name)
        image = Image.open(image_path).resize([SIDE, SIDE])
        image = np.array(image).astype(np.float32) / 128 - 1
        imagelist.append(image)

    return np.array(imagelist)


def load_graph(model_name):
    """Load graph"""
    with tf.io.gfile.GFile(model_name, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())
        tf.import_graph_def(graph_def, name='')


def main(): # pylint: disable=R0914
    """
    Before run this script, please check whether the following files
    exist in the same directory.
        classification.jpg
        folder 'calibration' contains 32 images
    :return: None
    """
    model_file = os.path.join(PATH, 'model/resnet_v1_50.pb')
    load_graph(model_file)
    graph = tf.compat.v1.get_default_graph()

    input_tensor = graph.get_tensor_by_name('input:0')
    output_tensor = graph.get_tensor_by_name('Reshape_1:0')

    image_path = os.path.join(PATH, 'data/classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float32) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])

    print('inference with origin pb********************')
    with tf.compat.v1.Session() as session:
        origin_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    config_file = os.path.join(OUTPUTS, 'config.json')
    record_file = os.path.join(OUTPUTS, 'record.txt')
    amct.create_quant_config(config_file, graph, batch_num=1, config_defination='./src/nuq_conf/nuq_quant.cfg')
    amct.quantize_model(graph, config_file, record_file)

    calibration_path = os.path.join(PATH, 'data/calibration')
    batch = load_image(calibration_path)

    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run(output_tensor, feed_dict={input_tensor: batch})

    amct.save_model(model_file, ['Reshape_1'], record_file, os.path.join(OUTPUTS, 'resnet-50_v1'))

    # reload and test the quantized model for 'Fakequant'.
    model_file = os.path.join(OUTPUTS, 'resnet-50_v1_quantized.pb')
    with tf.io.gfile.GFile(model_file, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.compat.v1.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def_reload, name='')

    print('inference with quantized pb====================')
    with tf.compat.v1.Session(graph=graph_reload) as session:
        fakequant_prediction = session.run('Reshape_1:0', feed_dict={'input:0': image_test})

    print(
        'Origin Model Prediction:\n',
        '\tcategory index: %d\n' % origin_prediction.argmax(),
        '\tcategory prob: %.3f\n' % round(origin_prediction.max(), 3), end='')
    print(
        'Quantized Model Prediction:\n',
        '\tcategory index: %d\n' % fakequant_prediction.argmax(),
        '\tcategory prob: %.3f\n' % round(fakequant_prediction.max(), 3), end='')


if __name__ == '__main__':
    main()
