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
# "info", "debug", "warning" and "error". The default level is "info".
import amct_tensorflow as amct # pylint: disable=E0401
amct.set_logging_level(print_level='info', save_level='info')


SIDE = 224
PATH = os.path.realpath('./')
MODEL_DIR = os.path.join(PATH, 'model')
OUTPUTS = os.path.join(PATH, 'outputs/convert_qat')
INPUT = 'input'
PREDICTION = 'MobilenetV2/Predictions/Softmax'


def main(): # pylint: disable=too-mant-locals, not-context-manager
    """main"""
    # Step one, inferece the original model to obtain the original
    # precision before conversion.
    model_path = os.path.join(MODEL_DIR, 'mobilenetv2_qat.pb')
    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    input_tensor = graph.get_tensor_by_name(INPUT + ':0')
    output_tensor = graph.get_tensor_by_name(PREDICTION + ':0')

    image_path = os.path.join(PATH, 'data/convert_qat.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float32) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])

    with tf.compat.v1.Session() as session:
        origin_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    # Step two, convert QAT pb_model to ascend quantized model and save
    # it. The ascend quantized model can be used for simulating test on
    # CPU/GPU and evaluate the accuracy of quantized model, and it can
    # also be used for ATC to generate the model running on Ascend AI
    # Processor.
    record_path = os.path.join(OUTPUTS, 'record.txt')
    amct.convert_qat_model(
        pb_model=model_path, outputs=[PREDICTION], save_path=os.path.join(OUTPUTS, 'mobilenet_v2'),
        record_file=record_path)

    # Step three, reload and test the quantized model.
    model_path = os.path.join(OUTPUTS, 'mobilenet_v2_quantized.pb')
    with tf.io.gfile.GFile(name=model_path, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def=graph_def_reload, name='')

    input_tensor = graph_reload.get_tensor_by_name(INPUT + ':0')
    output_tensor = graph_reload.get_tensor_by_name(PREDICTION + ':0')
    with tf.compat.v1.Session(graph=graph_reload) as session:
        convert_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    print(
        'Origin Model Prediction:\n',
        '\tcategory index: {}\n'.format(origin_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(origin_prediction.max()))
    print(
        'Quantized Model Prediction:\n',
        '\tcategory index: {}\n'.format(convert_prediction.argmax()),
        '\tcategory prob: {:.3f}\n'.format(convert_prediction.max()))


if __name__ == '__main__':
    main()
