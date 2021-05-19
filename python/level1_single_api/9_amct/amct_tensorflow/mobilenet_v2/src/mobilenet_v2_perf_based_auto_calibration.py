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
from typing import Sized
import numpy as np
from PIL import Image
import tensorflow as tf

# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct
amct.set_logging_level(print_level='info', save_level='info')


SIDE = 224
PATH = os.path.realpath('./')
OUTPUTS = os.path.join(PATH, 'outputs/perf_based_auto_calibration')


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='MobileNet V2 DEMO')
    parser.add_argument(
        '--sampler_config_file', dest='sampler_config_file', type=str, help='The simple configure define file.',
        default='./src/perf_conf/mobilenet_v2_sampler_config.cfg')
    parser.add_argument(
        '--cfg_define', dest='cfg_define', type=str, help='The simple configure define file.', default=None)

    return parser.parse_args()


def load_image(path):
    """Load images to memory"""
    imagelist = []
    path = os.path.realpath(path)
    for file_name in os.listdir(path):
        image_path = os.path.join(path, file_name)
        image = Image.open(image_path).resize([SIDE, SIDE])
        image = np.array(image).astype(np.float32) / 128 - 1
        imagelist.append(image)

    return np.array(imagelist)


def load_image_test():
    """Load test iamge to memory"""
    image_path = os.path.join(PATH, 'data/classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float32) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])
    return image_test


def load_model(model_path, input_tensor_name, output_tensor_name):
    """Load model and parser model to memory"""
    tf.compat.v1.reset_default_graph()
    model_path = os.path.realpath(model_path)
    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    input_tensor = graph.get_tensor_by_name(input_tensor_name)
    output_tensor = graph.get_tensor_by_name(output_tensor_name)
    return graph, input_tensor, output_tensor


def get_model_prediction(model_path, input_tensor_name, output_tensor_name):
    """Run model and get prediction of the model"""
    _, input_tensor, output_tensor = load_model(model_path, input_tensor_name, output_tensor_name)

    image_test = load_image_test()

    with tf.compat.v1.Session() as session:
        model_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    return model_prediction


def quantize_all_layers(model_path, input_tensor_name, output_tensor_name):
    """Quantify all layers of the whole network."""
    # Step one, load original model.
    graph, input_tensor, output_tensor = load_model(model_path, input_tensor_name, output_tensor_name)

    # Step two, generate the quantization config file.
    config_path = os.path.join(OUTPUTS, 'config.json')
    if ARGS.cfg_define is not None:
        amct.create_quant_config(
            config_file=config_path, graph=graph, skip_layers=[], batch_num=1, config_defination=ARGS.cfg_define)
    else:
        amct.create_quant_config(config_file=config_path, graph=graph, skip_layers=[], batch_num=1)

    # Step three, quantize the model.
    record_path = os.path.join(OUTPUTS, 'record.txt')
    amct.quantize_model(graph=graph, config_file=config_path, record_file=record_path)

    # Step four, calibrate and save the quantized model.
    calibration_path = os.path.join(PATH, 'data/calibration')
    batch = load_image(calibration_path)
    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run(output_tensor, feed_dict={input_tensor: batch})
    amct.save_model(
        pb_model=model_path, outputs=['MobilenetV2/Predictions/Reshape_1'], record_file=record_path,
        save_path=os.path.join(OUTPUTS, 'mobilenet_v2_all_layers'))


def accuracy_analysis(origin_prediction, fakequant_prediction):
    """Print the accuracy before and after quantization."""
    print(
        'Origin Model Prediction:\n',
        '\tcategory index: {}\n'.format(origin_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(round(origin_prediction.max(), 3)),
        end='')
    print(
        'Quantized Model Prediction:\n',
        '\tcategory index: {}\n'.format(fakequant_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(round(fakequant_prediction.max(), 3)),
        end='')


def main():
    """main precess"""
    original_model_path = os.path.join(PATH, 'model/mobilenetv2_tf.pb')
    quantized_model_path = os.path.join(OUTPUTS, 'mobilenet_v2_all_layers_quantized.pb')
    input_tensor_name = 'input:0'
    output_tensor_name = 'MobilenetV2/Predictions/Reshape_1:0'

    if not os.path.exists(quantized_model_path):
        quantize_all_layers(original_model_path, input_tensor_name, output_tensor_name)

    sampler_config_file = ARGS.sampler_config_file
    save_dir = os.path.join(OUTPUTS, 'mobilenet_v2')
    amct.perf_based_auto_calibration(
        original_model_path, quantized_model_path, [output_tensor_name[: -2]], sampler_config_file, save_dir)

    result_model_path = os.path.join(OUTPUTS, 'mobilenet_v2_quantized.pb')
    origin_prediction = get_model_prediction(original_model_path, input_tensor_name, output_tensor_name)
    fakequant_prediction = get_model_prediction(result_model_path, input_tensor_name, output_tensor_name)
    accuracy_analysis(origin_prediction, fakequant_prediction)


if __name__ == '__main__':
    ARGS = parse_args()
    main()
