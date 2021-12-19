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
import sys
import os
import numpy as np
import acl
import cv2
import time
import asyncore
import pickle
import socket
import struct

sys.path.append("../../../common")
from acllite_resource import AclLiteResource
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage

# config for ascend detection
LABELS = ["ascend"]
MODEL_PATH = "../model/yolov3_ascend_logo.om"

# yolov3 input image size
MODEL_WIDTH = 416
MODEL_HEIGHT = 416

STRIDE_LIST = [8, 16, 32]

# object detection parameters
CONF_THRESHOLD = 0.3
IOU_THRESHOLD = 0.45
CLASS_NUM = len(LABELS)
NUM_CHANNEL = 3 * (CLASS_NUM + 5)

# color for show detection results
COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]

# RealSense D435 Ethernet config
print('Number of arguments:', len(sys.argv), 'arguments.')
print('Argument List:', str(sys.argv))
LOCAL_IP_ADDRESS = '192.168.8.136'  # 200DK ip
MC_IP_ADDRESS = '192.168.8.102'  # PC ip
PORT = 1024
CHUNK_SIZE = 4096


class ImageClient(asyncore.dispatcher):
    """
    UDP client for each camera server
    """

    def __init__(self, server, source):
        asyncore.dispatcher.__init__(self, server)
        self._address = server.getsockname()[0]
        self._port = source[1]
        self._buffer = bytearray()
        self._window_name = self._port
        self._remaining_bytes = 0
        self._frame_id = 0
        self._image_data = np.array([])
        self._frame_data = b''
        self._frame_length = 0
        self._timestamp = 0

    def handle_read(self):
        """
        Read data
        :return: null
        """

        if self._remaining_bytes == 0:
            # get the expected frame size
            self._frame_length = struct.unpack('<I', self.recv(4))[0]

            # get the timestamp of the current frame
            self._timestamp = struct.unpack('<d', self.recv(8))
            self._remaining_bytes = self._frame_length

        # request the frame data until the frame is completely in buffer
        receive_frame_data = self.recv(self._remaining_bytes)
        self._buffer += receive_frame_data
        self._remaining_bytes -= len(receive_frame_data)

        # once the frame is fully received, process/display it
        if len(self._buffer) == self._frame_length:
            self.handle_frame()

    def handle_frame(self):
        """
        Execute model and send result
        :return: null
        """

        # convert the frame from string to numerical data
        self._image_data = pickle.loads(self._buffer)
        # print(self._image_data.shape)
        self._buffer = bytearray()
        self._frame_id += 1

        # yolov3 model inference with image data
        yolov3_inference_result = inference(yolov3_model, self._image_data)

        # send inference result from 200DK to PC
        send_result_data = pickle.dumps(yolov3_inference_result)

        # capture the length of the data portion of the message
        data_length = struct.pack('<I', len(send_result_data))

        # for the message transmission
        self._frame_data = b''.join([data_length, send_result_data])
        self.send(self._frame_data)

    def get_img(self):
        """
        Get image data
        :return: self._image_data
        """

        return self._image_data

    def readable(self):
        """
        Readable or not
        :return: True
        """

        return True


class EtherSenseClient(asyncore.dispatcher):
    """
    UDP server
    """

    def __init__(self):
        asyncore.dispatcher.__init__(self)
        self._server_address = (LOCAL_IP_ADDRESS, PORT)

        # create a socket for TCP connection between the client and server
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(5)
        self.bind(self._server_address)
        self.listen(10)

        self._image_data = np.array([])
        self.handler = 0

    def writable(self):
        """
        Don't want write notifies
        :return: False
        """

        return False

    def readable(self):
        """
        Readable or not
        :return: True
        """

        return True

    def handle_connect(self):
        """
        Print UDP connection messages
        :return: null
        """

        print("connection recvied")

    def handle_accept(self):
        """
        Print UDP connection messages and receive data
        :return: null
        """

        pair = self.accept()
        if pair is not None:
            sock_, addr_ = pair
            print('Incoming connection from %s' % repr(addr_))

            # when a connection is attempted, delegate image receival to the ImageClient
            self.handler = ImageClient(sock_, addr_)
            self._image_data = self.handler.get_img()


def preprocess_cv2(original_bgr_img):
    """
    Preprocess cv2 bgr image for yolov3 input
    :param original_bgr_img: original image with BGR format
    :return: processed image, MODEL_WIDTH, MODEL_HEIGHT
    """

    shape = original_bgr_img.shape[:2]  # [height, width]

    net_h = MODEL_HEIGHT
    net_w = MODEL_WIDTH
    scale = min(float(net_h) / float(shape[0]), float(net_w) / float(shape[1]))
    new_h = int(shape[0] * scale)
    new_w = int(shape[1] * scale)

    dw = (net_w - new_w) / 2
    dh = (net_h - new_h) / 2

    # yolov3 isobi scaling
    img = cv2.resize(original_bgr_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0))

    return img, shape[1], shape[0]


def overlap(xy_min1, xy_max1, xy_min2, xy_max2):
    """
    Calculate the width/height of overlap area between bbox1 and bbox2
    :param xy_min1: x_min or y_min of bbox1
    :param xy_max1: x_max or y_max of bbox1
    :param xy_min2: x_min or y_min of bbox2
    :param xy_max2: x_max or y_max of bbox2
    :return:
    """

    left = max(xy_min1, xy_min2)
    right = min(xy_max1, xy_max2)

    return right - left


def cal_iou(prediction_box, ground_truth_box):
    """
    Calculate IOU between box and truth
    :param prediction_box: model prediction bounding box: [x_min, y_min, x_max, y_max]
    :param ground_truth_box: ground truth bounding box: [x_min, y_min, x_max, y_max]
    :return: IOU between prediction_box and ground_truth_box
    """

    overlap_w = overlap(prediction_box[0], prediction_box[2], ground_truth_box[0], ground_truth_box[2])
    overlap_h = overlap(prediction_box[1], prediction_box[3], ground_truth_box[1], ground_truth_box[3])

    if overlap_w <= 0 or overlap_h <= 0:
        return 0

    inter_area = overlap_w * overlap_h
    union_area = (prediction_box[2] - prediction_box[0]) * (prediction_box[3] - prediction_box[1]) + \
                 (ground_truth_box[2] - ground_truth_box[0]) * (ground_truth_box[3] - ground_truth_box[1]) - inter_area

    return inter_area * 1.0 / union_area


def apply_nms(original_boxes, thresh):
    """
    Hard nms for yolov3 output boxes's postprocess
    :param original_boxes: yolov3 output boxes
    :param thresh: nms thresh hold
    :return: nms result list
    """

    nms_result = []

    for class_num in range(CLASS_NUM):
        one_class_boxes = original_boxes[class_num]
        sorted_boxes = sorted(one_class_boxes, key=lambda d: d[5])[::-1]

        result_box_id = dict()
        for box_id in range(len(sorted_boxes)):
            if box_id in result_box_id:
                continue

            truth = sorted_boxes[box_id]
            for box_id_else in range(box_id + 1, len(sorted_boxes)):
                if box_id_else in result_box_id:
                    continue
                box_else = sorted_boxes[box_id_else]
                iou = cal_iou(box_else, truth)
                if iou >= thresh:
                    result_box_id[box_id_else] = 1

        for box_id in range(len(sorted_boxes)):
            if box_id not in result_box_id:
                nms_result.append(sorted_boxes[box_id])

    return nms_result


def decode(conv_output, img_w, img_h):
    """
    Decode 3 output feature maps to object detection result
    :param conv_output: 3 output feature maps
    :param img_w: original image width
    :param img_h: original image height
    :return: object detection result
    """

    conv_output_h, conv_output_w, _ = conv_output.shape
    feature_map = conv_output.reshape((conv_output_h * conv_output_w, 3, 5 + CLASS_NUM))
    resize_ratio = min(MODEL_WIDTH / img_w, MODEL_HEIGHT / img_h)
    dw = (MODEL_WIDTH - resize_ratio * img_w) / 2
    dh = (MODEL_HEIGHT - resize_ratio * img_h) / 2

    bbox = np.zeros((conv_output_h * conv_output_w, 3, 4))
    bbox[..., 0] = np.maximum((feature_map[..., 0] - feature_map[..., 2] / 2.0 - dw) / resize_ratio, 0)  # x_min
    bbox[..., 1] = np.maximum((feature_map[..., 1] - feature_map[..., 3] / 2.0 - dh) / resize_ratio, 0)  # y_min
    bbox[..., 2] = np.minimum((feature_map[..., 0] + feature_map[..., 2] / 2.0 - dw) / resize_ratio, img_w)  # x_max
    bbox[..., 3] = np.minimum((feature_map[..., 1] + feature_map[..., 3] / 2.0 - dh) / resize_ratio, img_h)  # y_max

    feature_map[..., :4] = bbox
    feature_map = feature_map.reshape((-1, 5 + CLASS_NUM))
    feature_map[:, 4] = feature_map[:, 4] * feature_map[:, 5:].max(1)
    feature_map = feature_map[feature_map[:, 4] >= CONF_THRESHOLD]
    feature_map[:, 5] = np.argmax(feature_map[:, 5:], axis=-1)

    all_boxes = [[]]
    for box_index in range(feature_map.shape[0]):
        each_box = [int(feature_map[box_index, iy]) for iy in range(4)]
        each_box.append(int(feature_map[box_index, 5]))
        each_box.append(feature_map[box_index, 4])
        all_boxes[each_box[4] - 1].append(each_box)

    return all_boxes


def convert_labels(label_list):
    """
    Convert label index to class name
    :param label_list: index number of label
    :return: class name of label
    """

    label_names = []
    if isinstance(label_list, np.ndarray):
        label_list = label_list.tolist()
        label_names = [LABELS[int(index)] for index in label_list]

    return label_names


def post_process(infer_output, img_w, img_h):
    """
    Convert yolov3 output feature maps to detection result
    :param infer_output: 3 output feature maps
    :param img_w: original image width
    :param img_h: original image height
    :return: object detetion result with detection_classes, detection_boxes, detection_scores
    """

    result_dict = dict()
    all_boxes = [[]]
    for feature_map_id in range(3):
        feature_map = infer_output[feature_map_id].reshape(
            (MODEL_HEIGHT // STRIDE_LIST[feature_map_id], MODEL_WIDTH // STRIDE_LIST[feature_map_id], NUM_CHANNEL))
        boxes = decode(feature_map, img_w, img_h)
        all_boxes = [all_boxes[iy] + boxes[iy] for iy in range(CLASS_NUM)]
    nms_result = apply_nms(all_boxes, IOU_THRESHOLD)
    if not nms_result:
        result_dict['detection_classes'] = []
        result_dict['detection_boxes'] = []
        result_dict['detection_scores'] = []
    else:
        nms_result_array = np.array(nms_result)
        picked_boxes = nms_result_array[:, 0:4]
        picked_boxes = picked_boxes[:, [1, 0, 3, 2]]
        picked_classes = convert_labels(nms_result_array[:, 4])
        picked_score = nms_result_array[:, 5]
        result_dict['detection_classes'] = picked_classes
        result_dict['detection_boxes'] = picked_boxes.tolist()
        result_dict['detection_scores'] = picked_score.tolist()

    return result_dict


def inference(model, bgr_img):
    """
    Yolov3 model inference pipeline
    :param model: yolov3 om model
    :param bgr_img: opencv bgr image
    :return: yolov3 detection result
    """

    if bgr_img.shape[0] > 0:
        t_preprocess = 0
        t_inference = 0
        t_post_process = 0

        # preprocess image
        t1 = time.time()
        processed_bgr_img, img_w, img_h = preprocess_cv2(bgr_img)
        t2 = time.time()
        t_preprocess += (t2 - t1)

        # model inference
        inference_result_list = model.execute([processed_bgr_img, ])
        t3 = time.time()
        t_inference += (t3 - t2)

        # post process
        inference_result = post_process(inference_result_list, img_w, img_h)
        t4 = time.time()
        t_post_process += (t4 - t3)

        print("*" * 40)
        print("result = ", inference_result)
        print("preprocess cost:", t2 - t1)
        print("forward cost:", t3 - t2)
        print("post process cost:", t4 - t3)
        print("FPS:", 1 / (t4 - t1))
        print("*" * 40)

        return inference_result
    else:
        return dict()


if __name__ == "__main__":
    # acl resource init
    acl_resource = AclLiteResource()
    acl_resource.init()

    # load om model
    yolov3_model = AclLiteModel(MODEL_PATH)

    # send the multicast message
    multicast_group = (LOCAL_IP_ADDRESS, PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connections = {}

    try:
        # Send data to the multicast group
        print('sending "%s"' % 'EtherSensePing' + str(multicast_group))
        sent = sock.sendto('EtherSensePing'.encode(), multicast_group)

        # defer waiting for a response using Asyncore
        client = EtherSenseClient()
        # print("data shape:", client._image_data.shape)
        asyncore.loop()

    except socket.timeout:
        print('timed out, no more responses')
    finally:
        print(sys.stderr, 'closing socket')
        sock.close()
