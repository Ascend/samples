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
from PIL import Image
import tensorflow as tf

# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct
amct.set_logging_level(print_level='info', save_level='info')


SIDE = 224
PATH = os.path.realpath('./')
OUTPUTS = 'outputs/convert_model'


def load_image(path):
    """load image"""
    imagelist = []

    for file in os.listdir(path):
        image_path = os.path.join(path, file)
        image = Image.open(image_path).resize([224, 224])
        image = np.array(image).astype(np.float) / 128 - 1
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
    model_path = os.path.join(PATH, 'model/mobilenet_v2_1.0_224_frozen.pb')

    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    input_tensor = graph.get_tensor_by_name('input:0')
    output_tensor = graph.get_tensor_by_name(
        'MobilenetV2/Predictions/Reshape_1:0')

    image_path = os.path.join(PATH, 'data/classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])

    with tf.compat.v1.Session() as session:
        origin_prediction = session.run(output_tensor, feed_dict={input_tensor: image_test})

    # Step two, convert pb_model to quantized model and save it. To finish
    # this work, a record_file containing scale and offset is essential. The
    # quantized model can be used for simulating test on CPU/GPU and evaluate
    # the accuracy of quantized model, and it can also be used for ATC to
    # generate the model running on Ascend AI Processor.
    record_path = os.path.join(PATH, 'data/record_quantized.txt')
    amct.convert_model(
        pb_model=model_path, outputs=['MobilenetV2/Predictions/Reshape_1'],
        record_file=record_path, save_path=os.path.join(PATH, OUTPUTS, 'mobilenet_v2'))

    # Step three, reload and test the quantized model for 'Fakequant'.
    model_path = os.path.join(PATH, OUTPUTS, 'mobilenet_v2_quantized.pb')
    with tf.io.gfile.GFile(name=model_path, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def=graph_def_reload, name='')

    with tf.compat.v1.Session(graph=graph_reload) as session:
        fakequant_prediction = session.run('MobilenetV2/Predictions/Reshape_1:0', feed_dict={'input:0': image_test})

    print('Origin Model Prediction:\n',
          '\tcategory index: %d\n' % origin_prediction.argmax(),
          '\tcategory prob: %.3f\n' % round(origin_prediction.max(), 3),
          end='')
    print('Quantized Model Prediction:\n',
          '\tcategory index: %d\n' % fakequant_prediction.argmax(),
          '\tcategory prob: %.3f\n' % round(fakequant_prediction.max(), 3),
          end='')


if __name__ == '__main__':
    main()
