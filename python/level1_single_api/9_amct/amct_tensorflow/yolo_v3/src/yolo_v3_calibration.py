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
import numpy as np
import tensorflow as tf # pylint: disable=E0401
from PIL import Image # pylint: disable=E0401
from PIL import ImageDraw # pylint: disable=E0401

# Import AMCT toolkit and set log level. There are four log levels,
# 'info', 'debug', 'warning' and 'error'. The default level is 'info'.
import amct_tensorflow as amct # pylint: disable=E0401
amct.set_logging_level(print_level='info', save_level='info')


PATH, _ = os.path.realpath('./')
OUTPUTS = os.path.join(PATH, 'outputs')
SIDE = 416
INPUT_NAME = 'input'
XY_NAME = 'concat_9'
CONF_NAME = 'concat_7'
PROB_NAME = 'concat_8'


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


def main(): # pylint: disable=too-many-statements, too-many-locals, not-context-manager
    """
    Before run this script, please check whether the following files
    exist in the same directory.
        calibration.jpg
        COCO_labels.txt,
        detection.jpg,
        yolov3_tensorflow_1.5.pb,
    :return: None
    """

    # Step one, load the trained model.
    # This sample will use YOLOv3 which is trained with COCO dataset
    # and save as 'yolov3_tensorflow_1.5.pb'. Therefore, loading COCO
    # labels and test image to do inference.
    model_path = os.path.join(PATH, 'model/yolov3_tensorflow_1.5.pb')
    with tf.io.gfile.GFile(model_path, mode='rb') as model:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(model.read())

    tf.import_graph_def(graph_def, name='')
    graph = tf.compat.v1.get_default_graph()
    input_tensor = graph.get_tensor_by_name(INPUT_NAME + ':0')
    xy_tensor = graph.get_tensor_by_name(XY_NAME + ':0')
    confidence_tensor = graph.get_tensor_by_name(CONF_NAME + ':0')
    probability_tensor = graph.get_tensor_by_name(PROB_NAME + ':0')

    labels = []
    label_path = os.path.join(PATH, 'data/COCO_labels.txt')
    with open(label_path) as label_file:
        for line in label_file:
            labels.append(line[: -1])

    image_path = os.path.join(PATH, 'data/detection.jpg')
    image_input = preprocessing(image_path)
    image_input = image_input.reshape([1, SIDE, SIDE, 3])

    with tf.compat.v1.Session() as session:
        xy, confidence, probability = session.run(
            [xy_tensor, confidence_tensor, probability_tensor], feed_dict={input_tensor: image_input})

    xy = xy.reshape([-1, 4])
    confidence = confidence.reshape([-1, 1])
    probability = probability.reshape([-1, 80])
    bbox_origin = np.concatenate((xy, confidence, probability), axis=-1)

    # Step two, generate the quantization config file.
    # The function 'create_quant_config' will generate a config file
    # that describe how to quantize the model in graph. The config file
    # is saved as JSON format, and you can edit the file to configurate
    # your quantization parameters of each layers(support FC, CONV and
    # DW) easily.
    config_path = os.path.join(OUTPUTS, 'config.json')
    cfg_define = os.path.join(PATH, 'src/yolo_quant.cfg')
    amct.create_quant_config(config_file=config_path, graph=graph, config_defination=cfg_define)

    # Step three, quantize the model.
    # The function 'quantize_model' will quantize your model in graph
    # according to config file.
    record_path = os.path.join(OUTPUTS, 'record.txt')
    amct.quantize_model(graph=graph, config_file=config_path, record_file=record_path)

    # Step four, calibrate and save the quantized model.
    # The quantization parameters require one or more batch data to do
    # inference and find the optimal values. This process is referred to
    # calibration. For example, we use 1 pictures for calibration. Be
    # sure to initialize the quantization parameters first. If your
    # model have variables, you must reload the checkpoint before
    # inference. After calibration, you can save the quantized model from
    # original model and record_file. The quantized model can be used for
    # simulating test on CPU/GPU and evaluate the accuracy of quantized
    # model. and it can also be used for ATC to generate the model
    # running on Ascend AI Processor.
    calibration_path = os.path.join(PATH, 'data/calibration.jpg')
    image_calibration = preprocessing(calibration_path)
    image_calibration = image_calibration.reshape([1, SIDE, SIDE, 3])

    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run([xy_tensor, confidence_tensor, probability_tensor], feed_dict={input_tensor: image_calibration})

    amct.save_model(
        pb_model=model_path, outputs=[XY_NAME, CONF_NAME, PROB_NAME], record_file=record_path,
        save_path=os.path.join(OUTPUTS, 'yolo_v3'))

    # Step five, reload and test the quantized model for 'Fakequant'.
    model_path = os.path.join(OUTPUTS, 'yolo_v3_quantized.pb')
    with tf.io.gfile.GFile(name=model_path, mode='rb') as model:
        graph_def_reload = tf.compat.v1.GraphDef()
        graph_def_reload.ParseFromString(model.read())

    graph_reload = tf.Graph()
    with graph_reload.as_default():
        tf.import_graph_def(graph_def=graph_def_reload, name='')
    input_tensor = graph_reload.get_tensor_by_name(INPUT_NAME + ':0')
    xy_tensor = graph_reload.get_tensor_by_name(XY_NAME + ':0')
    confidence_tensor = graph_reload.get_tensor_by_name(CONF_NAME + ':0')
    probability_tensor = graph_reload.get_tensor_by_name(PROB_NAME + ':0')

    with tf.compat.v1.Session(graph=graph_reload) as session:
        xy, confidence, probability = session.run(
            [xy_tensor, confidence_tensor, probability_tensor], feed_dict={input_tensor: image_input})

    xy = xy.reshape([-1, 4])
    confidence = confidence.reshape([-1, 1])
    probability = probability.reshape([-1, 80])
    bbox_fakequant = np.concatenate((xy, confidence, probability), axis=-1)

    bounds, classes, scores = postprocessing(bbox_origin, image_path)
    origin_image = annotate(image_path, bounds, classes, scores, labels)
    origin_image.save('origin.png', 'png')
    origin_image.show('origin')
    print('origin.png save successfully!')

    bounds, classes, scores = postprocessing(bbox_fakequant, image_path)
    quantize_image = annotate(image_path, bounds, classes, scores, labels)
    quantize_image.save('quantize.png', 'png')
    quantize_image.show('quantize')
    print('quantize.png save successfully!')


if __name__ == '__main__':
    main()
