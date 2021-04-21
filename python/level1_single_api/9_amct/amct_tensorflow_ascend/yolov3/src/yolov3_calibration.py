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
import tensorflow as tf # pylint: disable=E0401

# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct # pylint: disable=E0401
from yolov3_inference import inference_yolo_graph
from yolov3_inference import process_image_bbox

amct.set_logging_level(print_level='info', save_level='info')

SIDE = 416
CUR_DIR, _ = os.path.split(os.path.realpath(__file__))
PRE_MODEL_DIR = os.path.join(CUR_DIR, '../pre_model')
DATA_DIR = os.path.join(CUR_DIR, '../data')
RESULT_DIR = os.path.join(CUR_DIR, '../result')
CALI_DIR = os.path.join(CUR_DIR, '../calibration_tmp')
TMP = os.path.join(CUR_DIR, '../tmp')


def parse_pb_to_graph(pb_model):
    """
    Parse pb model to graph
    :param pb_model: string, pb model's path
    :return graph: tf.Graph
    """
    with tf.io.gfile.GFile(pb_model, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    graph = tf.Graph()
    with graph.as_default():
        tf.import_graph_def(graph_def, name='')
    
    return graph


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


def compare_ori_and_quant():
    """
    比较量化模型和非量化模型的推理差异
    """
    image_path = DATA_DIR
    detection_image = os.path.join(image_path, 'detection.jpg')
    labels = []
    label_path = os.path.join(image_path, 'COCO_labels.txt')
    with open(label_path) as label_file:
        for line in label_file:
            labels.append(line[: -1])

    ori_model = os.path.join(PRE_MODEL_DIR, 'yolov3_tensorflow_1.5.pb')
    yolov3_inputs = 'input:0'
    yolov3_outputs = ['concat_9', 'concat_7', 'concat_8']
    graph = parse_pb_to_graph(ori_model)
    bbox_origin = inference_yolo_graph(graph=graph, inputs=yolov3_inputs, outputs=yolov3_outputs,
                                       image=detection_image, device='npu')
    origin_image = process_image_bbox(detection_image, bbox_origin, labels,
                                      file_name=os.path.join(CALI_DIR, 'origin.png'))
    print('origin.png save successfully!')

    quant_model = os.path.join(RESULT_DIR, 'yolov3_quantized.pb')
    quant_graph = parse_pb_to_graph(quant_model)
    bbox_fakequant = inference_yolo_graph(graph=quant_graph, inputs=yolov3_inputs, outputs=yolov3_outputs,
                                          image=detection_image, device='cpu')
    quantize_image = process_image_bbox(detection_image, bbox_fakequant, labels,
                                        file_name=os.path.join(CALI_DIR, 'quantize.png'))
    print('quantize.png save successfully!')

    origin_image.show('origin')
    quantize_image.show('quantize')


def main():
    """
    对yolov3模型做训练后量化的过程
    """
    # 解析pb模型为graph
    ori_model = os.path.join(PRE_MODEL_DIR, 'yolov3_tensorflow_1.5.pb')
    yolov3_inputs = 'input:0'
    yolov3_outputs = ['concat_9', 'concat_7', 'concat_8']

    graph = parse_pb_to_graph(ori_model)
    # 调用create_quant_config_ascend接口，生成量化配置文件
    config_path = os.path.join(CALI_DIR, 'config.json')
    cfg_define = os.path.join(CUR_DIR, 'yolov3_calibration.cfg')
    amct.create_quant_config_ascend(config_file=config_path,
                                    graph=graph,
                                    config_defination=cfg_define)

    # 调用quantize_model_ascend接口，在图中插入量化算子
    record_path = os.path.join(CALI_DIR, 'record.txt')
    calibration_graph, calibration_outputs = amct.quantize_model_ascend(
        graph=graph,
        config_file=config_path,
        record_file=record_path,
        outputs=yolov3_outputs)

    # 使用校准集，完成校准过程，确定量化因子
    image_calibration = os.path.join(DATA_DIR, 'calibration.jpg')
    inference_yolo_graph(graph=calibration_graph, inputs=yolov3_inputs,
                         outputs=calibration_outputs, image=image_calibration, device='npu')

    # 保存量化模型
    amct.save_model_ascend(pb_model=ori_model,
                           outputs=yolov3_outputs,
                           record_file=record_path,
                           save_path=os.path.join(RESULT_DIR, 'yolov3'))

    # 比较量化模型和非量化模型的推理差异
    compare_ori_and_quant()

    # 整理推理过程中生成的文件夹
    create_file_path(TMP)
    shutil.move('./check_result.tf.json', './tmp/check_result.tf.json')
    shutil.move('./fusion_result.json', './tmp/fusion_result.json')


if __name__ == '__main__':
    main()
