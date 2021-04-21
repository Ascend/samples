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
import numpy as np
import tensorflow as tf # pylint: disable=E0401
from tensorflow.core.protobuf.rewriter_config_pb2 import \
    RewriterConfig # pylint: disable=E0401
from PIL import Image # pylint: disable=E0401
from PIL import ImageDraw # pylint: disable=E0401

SIDE = 416
CUR_DIR, _ = os.path.split(os.path.realpath(__file__))
PRE_MODEL_DIR = os.path.join(CUR_DIR, '../pre_model')
DATA_DIR = os.path.join(CUR_DIR, '../data')
RESULT_DIR = os.path.join(CUR_DIR, '../result')
CALI_DIR = os.path.join(CUR_DIR, '../calibration_tmp')
TMP = os.path.join(CUR_DIR, '../tmp')


def nms(bounds, classes, scores): # pylint: disable=too-many-locals, no-member
    """
    The non-maximum suppression algorithm.

    :param bounds: the bounding vertices
    :param classes: class of bounding
    :param scores: confidence of bounding
    :return: the best bounding, class and confidence
    """
    best_bounds = []
    best_scores = []
    best_classes = []
    for i in list(np.unique(classes)):
        mask_class = classes == i
        bounds_class = bounds[mask_class, :]
        scores_class = scores[mask_class]
        while bounds_class.size > 0:
            max_index = np.argmax(scores_class)
            best_bound = bounds_class[max_index]
            best_bounds.append(best_bound)
            best_scores.append(scores_class[max_index])
            best_classes.append(i)
            bounds_class = np.delete(bounds_class, max_index, axis=0)
            scores_class = np.delete(scores_class, max_index)
            if bounds_class.size == 0:
                break
            best_area = (best_bound[2] - best_bound[0]) * (best_bound[3] - best_bound[1])
            areas = (bounds_class[:, 2] - bounds_class[:, 0]) * (bounds_class[:, 3] - bounds_class[:, 1])
            xmax = np.maximum(best_bound[0], bounds_class[:, 0])
            xmin = np.minimum(best_bound[2], bounds_class[:, 2])
            ymax = np.maximum(best_bound[1], bounds_class[:, 1])
            ymin = np.minimum(best_bound[3], bounds_class[:, 3])
            width = np.maximum(0, xmin - xmax + 1)
            height = np.maximum(0, ymin - ymax + 1)
            areas_intersection = width * height
            iou = areas_intersection / (best_area + areas - areas_intersection)
            mask_iou = iou < 0.45
            bounds_class = bounds_class[mask_iou, :]
            scores_class = scores_class[mask_iou]
    return best_bounds, best_classes, best_scores


def preprocessing(image_path, side=416):
    """
    This function is preprocessing for YOLOv3 input.

    The preprocessing including scaling(scale the longer side of the
    image to parameter 'side'), padding(pad the scaled image with
    neutral grey to a square that side length equal to parameter
    'side') and normalization.

    :param image_path: a string of image path
    :param side: the side length of YOLOv3's input
    :return: a numpy array of processed image
    """

    image = Image.open(image_path)
    width, height = image.size
    scale = side / max([width, height])
    width_scaled = int(width * scale)
    height_scaled = int(height * scale)
    image_scaled = image.resize((width_scaled, height_scaled),
                                resample=Image.LANCZOS)
    image_array = np.array(image_scaled, dtype=np.float32)
    image_padded = np.full([side, side, 3], 128, dtype=np.float32)
    width_offset = (side - width_scaled) // 2
    height_offset = (side - height_scaled) // 2
    image_padded[
        height_offset:height_offset + height_scaled, width_offset:width_offset + width_scaled, :] = image_array
    image_norm = image_padded / 255
    return image_norm


def postprocessing(bbox, image_path, side=416, threshold=0.3): # pylint: disable=R0914
    """
    This function is postprocessing for YOLOv3 output.

    Before calling this function, reshape the raw output of YOLOv3 to
    following form
        numpy.ndarray:
            [
                x_min,
                y_min,
                x_max,
                y_max,
                confidence,
                probability of 80 classes
            ]
        shape: (-1, 85)
    The postprocessing restore the bounding rectangles of YOLOv3 output
    to origin scale and filter with non-maximum suppression.

    :param bbox: a numpy array of the YOLOv3 output
    :param image_path: a string of image path
    :param side: the side length of YOLOv3's input
    :param threshold: the threshold of non-maximum suppression
    :return: three list for best bound, class and score
    """

    bounds = bbox[:, 0: 4]
    confidence = bbox[:, 4]
    probability = bbox[:, 5:]

    image = Image.open(image_path)
    width, height = image.size
    scale = side / max([width, height])
    width_scaled = int(width * scale)
    height_scaled = int(height * scale)
    width_offset = (side - width_scaled) // 2
    height_offset = (side - height_scaled) // 2
    bounds[:, (0, 2)] = (bounds[:, (0, 2)] - width_offset) / scale
    bounds[:, [1, 3]] = (bounds[:, [1, 3]] - height_offset) / scale
    bounds = bounds.astype(np.int32)

    bounds[np.where(bounds < 0)] = 0
    bounds[np.where(bounds[:, 2] > width), 2] = width - 1
    bounds[np.where(bounds[:, 3] > height), 3] = height - 1
    mask = np.ones(bounds.shape, dtype=bool)
    mask[:, 2] = (bounds[:, 2] - bounds[:, 0]) > 0
    mask[:, 3] = (bounds[:, 3] - bounds[:, 1]) > 0
    mask = np.logical_and.reduce(mask, axis=1)
    classes = np.argmax(probability, axis=1)
    scores = confidence * probability[np.arange(classes.size), classes]
    mask = mask & (scores > threshold)
    bounds = bounds[mask]
    classes = classes[mask]
    scores = scores[mask]
    return nms(bounds, classes, scores)


def annotate(image_path, bounds, classes, scores, labels):
    """
    This function will draw the bounding rectangles, classes and scores
    on the image and return it.

    :param image_path: a string of image path
    :param bounds: a list of vertex coordinates
    :param classes: a list of class number
    :param scores: a list of confidence
    :param labels: a list of COCO dataset label
    :return: a boolean whether saving successful
    """
    image = Image.open(image_path)
    draw = ImageDraw.Draw(image)
    for bound, cls, score in zip(bounds, classes, scores):
        draw.rectangle([(bound[0], bound[1]), (bound[2], bound[3])], None, (0, 255, 255))
        text = str(labels[cls]) + str(round(score, 3))
        text_size = draw.textsize(text)
        draw.rectangle([(bound[0], bound[1]), (bound[0] + text_size[0], bound[1] + text_size[1])], (0, 255, 255))
        draw.text((bound[0], bound[1]), text, (0, 0, 0))
    return image


def process_image_bbox(image, bbox, labels, file_name):
    """
    process image with detection result and save it
    :param image: string, image's path
    :param bbox: np.array, detection result from yolov3 inference
    :param labels: list of COCO dataset label
    :param file_name: string, path to save image with detection result
    :return image_processed: image with detection result
    """
    bounds, classes, scores = postprocessing(bbox, image)
    image_processed = annotate(image, bounds, classes, scores, labels)
    image_processed.save(file_name, 'png')
    return image_processed


def load_image(path):
    """
    Load image to memory.
    :param path: string, image's path
    :return : image array
    """
    imagelist = []

    for image_file in os.listdir(path):
        image_path = os.path.join(path, image_file)
        image = Image.open(image_path).resize([224, 224])
        image = np.array(image).astype(np.float) / 128 - 1
        imagelist.append(image)

    return np.array(imagelist)


def prepare_config(device='npu'):
    """
    prepare session's config for different device.
    :param device: a string, 'npu' and 'cpu' is valid
    :return config: config for tf.session
    """
    if device == 'npu':
        # config for Ascend processor
        config = tf.ConfigProto()
        custom_op = config.graph_options.rewrite_options.custom_optimizers.add()
        custom_op.name = "NpuOptimizer"
        custom_op.parameter_map["use_off_line"].b = True
        custom_op.parameter_map["precision_mode"].s = tf.compat.as_bytes("force_fp16")
        custom_op.parameter_map["graph_run_mode"].i = 0
        config.graph_options.rewrite_options.remapping = RewriterConfig.OFF
        custom_op.parameter_map["debug_dir"].s = tf.compat.as_bytes(str(TMP))
    else:
        config = tf.ConfigProto()
    return config


def inference_yolo_graph(graph, inputs, outputs, image, device='npu'):
    """
    do inference fo yolov3 graph
    :param graph: tf.Graph, yolov3's graph
    :param inputs: string, graph's input name
    :param outputs: list, graph's output
    :param image: string, path of image to do inference
    :param device: string, on which divice to do inference
    :return bbox_origin: np.array, result of inference
    """
    image_input = preprocessing(image)
    image_input = image_input.reshape([1, SIDE, SIDE, 3])
    outputs = [out + ':0' for out in outputs] # 获取tensor的输出
    with tf.compat.v1.Session(graph=graph, config=prepare_config(device)) as sess:
        outputs_val = sess.run(outputs, feed_dict={inputs: image_input})

    if len(outputs) == 3:
        sbbox, mbbox, lbbox = outputs_val
    else:
        sbbox, mbbox, lbbox = outputs_val[0:3]
    sbbox = sbbox.reshape([-1, 4])
    mbbox = mbbox.reshape([-1, 1])
    lbbox = lbbox.reshape([-1, 80])
    bbox_origin = np.concatenate((sbbox, mbbox, lbbox), axis=-1)

    return bbox_origin