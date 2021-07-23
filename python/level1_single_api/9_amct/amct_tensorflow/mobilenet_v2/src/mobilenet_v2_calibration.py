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


PATH = os.path.realpath('./')
OUTPUTS = os.path.join(PATH, 'outputs/calibration')
SIDE = 224
INPUT_NAME = 'input'
OUTPUT_NAME = 'MobilenetV2/Predictions/Reshape_1'


def load_image(path):
    """Load image to memory."""
    imagelist = []

    for file_name in os.listdir(path):
        image_path = os.path.join(path, file_name)
        image = Image.open(image_path).resize([SIDE, SIDE])
        image = np.array(image).astype(np.float32) / 128 - 1
        imagelist.append(image)

    return np.array(imagelist)


def main(): # pylint: disable=too-many-locals, not-context-manager
    """
    Before run this script, please check whether the following files
    exist in the same directory.
        classification.jpg,
        folder 'calibration' contains 32 images
    :return: None
    """

    # Step one, inference the original model to obtain the original
    # precision before quantization.
    model_path = os.path.realpath('./model/mobilenet_v2_1.0_224_frozen.pb')

    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    input_tensor = graph.get_tensor_by_name(INPUT_NAME + ':0')
    output_tensor = graph.get_tensor_by_name(OUTPUT_NAME + ':0')

    image_path = os.path.realpath('./data/classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float32) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])

    with tf.compat.v1.Session() as session:
        origin_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    # Step two, generate the quantization config file.
    # The function 'create_quant_config' will generate a config file
    # that describe how to quantize the model in graph. The config file
    # is saved as JSON format, and you can edit the file to configurate
    # your quantization parameters of each layers(support FC, CONV and
    # DW) easily.
    config_path = os.path.join(OUTPUTS, 'config.json')
    amct.create_quant_config(config_file=config_path, graph=graph, skip_layers=[], batch_num=1)

    # Step three, quantize the model.
    # The function 'quantize_model' will quantize your model in graph
    # according to config file.
    record_path = os.path.join(OUTPUTS, 'record.txt')
    amct.quantize_model(graph=graph, config_file=config_path, record_file=record_path)

    # Step four, calibrate and save the quantized model.
    # The quantization parameters require one or more batch data to do
    # inference and find the optimal values. This process is referred
    # to calibration. For example, we use 32 pictures for calibration.
    # Be sure to initialize the quantization parameters first. If your
    # model have variables, you must reload the checkpoint before
    # inference. After calibration, you can save the quantized model
    # from original model and record_file. The quantized model can be
    # used for simulating test on CPU/GPU and evaluate the accuracy of
    # quantized model, and it can also be used for ATC to generate the
    # model running on Ascend AI Processor.
    calibration_path = os.path.realpath('./data/calibration')
    batch = load_image(calibration_path)

    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run(output_tensor, feed_dict={input_tensor: batch})

    amct.save_model(
        pb_model=model_path, outputs=[OUTPUT_NAME], record_file=record_path,
        save_path=os.path.join(OUTPUTS, 'mobilenet_v2'))

    # Step five, reload and test the quantized model for 'Fakequant'.
    model_path = os.path.join(OUTPUTS, 'mobilenet_v2_quantized.pb')
    with tf.io.gfile.GFile(name=model_path, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def=graph_def_reload, name='')
    input_tensor = graph_reload.get_tensor_by_name(INPUT_NAME + ':0')
    output_tensor = graph_reload.get_tensor_by_name(OUTPUT_NAME + ':0')

    with tf.compat.v1.Session(graph=graph_reload) as session:
        fakequant_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

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
