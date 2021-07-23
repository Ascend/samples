#!/usr/bin/env python3 # pylint: disable=C0103
# -*- coding: UTF-8 -*-
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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import stat
import shutil
import argparse
import numpy as np
import tensorflow as tf # pylint: disable=C0411, E0401
from tensorflow.core.protobuf.rewriter_config_pb2 import \
    RewriterConfig # pylint: disable=E0401
from PIL import Image # pylint: disable=E0401


# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct # pylint: disable=E0401
amct.set_logging_level(print_level='info', save_level='info')


SIDE = 224
CUR_DIR, _ = os.path.split(os.path.realpath(__file__))
PRE_MODEL_DIR = os.path.join(CUR_DIR, '../pre_model')
DATA_DIR = os.path.join(CUR_DIR, '../data')
RESULT_DIR = os.path.join(CUR_DIR, '../result')
CALI_DIR = os.path.join(CUR_DIR, '../calibration_tmp')
TMP = os.path.join(CUR_DIR, '../tmp')

def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='mobilenetv2 demo')
    parser.add_argument('--cfg_define', 
                        dest='cfg_define',
                        help='The simple configure define file.',
                        default=None,
                        type=str)
    return parser.parse_args()


def load_image(path):
    """Load image to memory."""
    imagelist = []

    for image_file in os.listdir(path):
        image_path = os.path.join(path, image_file)
        image = Image.open(image_path).resize([224, 224])
        image = np.array(image).astype(np.float) / 128 - 1
        imagelist.append(image)

    return np.array(imagelist)


def create_file_path(file_dir):
    """
    Create path for file's dir
    :param file_dir: string, file's path
    :return: None 
    """
    mode = stat.S_IRWXU + stat.S_IRGRP + stat.S_IXGRP
    try:
        os.makedirs(file_dir, mode, exist_ok=True)
    except FileExistsError:
        pass


def prepare_config(device='npu'):
    """
    prepare session's config for different device.
    :param device: a string, 'npu' and 'cpu' is valid
    """
    if device == 'npu':
        # 昇腾AI处理器模型编译和优化配置
        config = tf.ConfigProto()
        custom_op = config.graph_options.rewrite_options.custom_optimizers.add()
        custom_op.name = "NpuOptimizer"
        # 配置1： 选择在昇腾AI处理器上执行推理run on Ascend NPU
        custom_op.parameter_map["use_off_line"].b = True
        # 配置2：在线推理场景下建议保持默认值force_fp16，使用float16精度推理，以获得较优的性能
        custom_op.parameter_map["precision_mode"].s = tf.compat.as_bytes("force_fp16")
        # 配置3：图执行模式，推理场景下请配置为0，训练场景下为默认1
        custom_op.parameter_map["graph_run_mode"].i = 0
        # 配置4：关闭remapping
        config.graph_options.rewrite_options.remapping = RewriterConfig.OFF
        # 配置5：设置dubug路径
        custom_op.parameter_map["debug_dir"].s = tf.compat.as_bytes(str(TMP))
    else:
        config = tf.ConfigProto()
    return config


def main(): # pylint: disable=too-many-locals, not-context-manager
    """
    Before run this script, please check whether the following files
    exist in the same directory.
        classification.jpg,
        folder 'calibration' contains 32 images
    :return: None
    """
    args = parse_args()
    create_file_path(TMP)

    # Step one, inference the original model to obtain the original
    # precision before quantization.
    model_path = os.path.join(PRE_MODEL_DIR, 'mobilenet_v2_1.0_224_frozen.pb')
    image_path = os.path.join(DATA_DIR, 'classification.jpg')
    image_test = Image.open(image_path).resize([SIDE, SIDE])
    image_test = np.array(image_test).astype(np.float) / 128 - 1
    image_test = image_test.reshape([1, SIDE, SIDE, 3])

    tf.reset_default_graph()
    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())
    tf.import_graph_def(graph_def, name='')

    graph = tf.compat.v1.get_default_graph()
    with tf.compat.v1.Session(config=prepare_config('npu')) as sess:
        origin_prediction = sess.run('MobilenetV2/Predictions/Reshape_1:0',
                                     feed_dict={'input:0': image_test})

    # Step two, generate the quantization config file.
    # The function 'create_quant_config' will generate a config file
    # that describe how to quantize the model in graph. The config file
    # is saved as JSON format, and you can edit the file to configurate
    # your quantization parameters of each layers(support FC, CONV and
    # DW) easily.
    cali_tmp_path = CALI_DIR
    config_path = os.path.join(cali_tmp_path, 'config.json')
    if args.cfg_define is not None:
        amct.create_quant_config_ascend(config_file=config_path,
                                        graph=graph,
                                        config_defination=args.cfg_define)
    else:
        amct.create_quant_config_ascend(config_file=config_path,
                                        graph=graph,
                                        skip_layers=[])

    # Step three, quantize the model.
    # The function 'quantize_model' will quantize your model in graph
    # according to config file.
    record_path = os.path.join(cali_tmp_path, 'record.txt')
    calibration_graph, calibration_outputs = amct.quantize_model_ascend(
        graph=graph,
        config_file=config_path,
        record_file=record_path,
        outputs=['MobilenetV2/Predictions/Reshape_1'])

    # Step four, calibrate and save the quantized model.
    # The quantization parameters require one batch data to do
    # inference and find the optimal values. This process is referred to
    # calibration. For example, we use 32 pictures for calibration. 
    # After calibration, you can save the quantized model from
    # original model and record_file. The quantized model can be used for
    # simulating test on CPU and evaluate the accuracy of quantized
    # model, and it can also be used for ATC to generate the model
    # running on Ascend AI Processor.
    batch = load_image(os.path.join(DATA_DIR, 'calibration'))

    tf.reset_default_graph()
    with calibration_graph.as_default():
        with tf.compat.v1.Session(config=prepare_config('npu')) as sess:
            sess.run(tf.compat.v1.global_variables_initializer())
            sess.run(calibration_outputs, feed_dict={'input:0': batch})

    amct.save_model_ascend(pb_model=model_path,
                           outputs=['MobilenetV2/Predictions/Reshape_1'],
                           record_file=record_path,
                           save_path=os.path.join(RESULT_DIR, 'MobileNetV2'))
    shutil.move('./check_result.tf.json', os.path.join(TMP, 'check_result.tf.json'))
    shutil.move('./fusion_result.json', os.path.join(TMP, 'fusion_result.json'))

    # Step five, reload and test the quantized model for 'Fakequant'.
    model_path = os.path.join(RESULT_DIR, 'MobileNetV2_quantized.pb')
    with tf.io.gfile.GFile(name=model_path, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def=graph_def_reload, name='')

    with tf.compat.v1.Session(graph=graph_reload, config=prepare_config('cpu')) as sess:
        fakequant_prediction = sess.run('MobilenetV2/Predictions/Reshape_1:0',
                                        feed_dict={'input:0': image_test})

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
