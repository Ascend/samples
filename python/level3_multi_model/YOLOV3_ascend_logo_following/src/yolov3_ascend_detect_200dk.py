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

sys.path.append("../../../common")
import os
import numpy as np
import acl
import cv2 as cv
import time
import sys, getopt
import asyncore
import pickle
import socket
import struct

from PIL import Image
from atlas_utils.constants import *
from atlas_utils.acl_resource import AclResource
from atlas_utils.utils import *
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage

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
LOCAL_IP_ADDRESS = '192.168.8.134'  # 200DK ip
MC_IP_ADDRESS = '192.168.8.102'  # PC ip
PORT = 1024
CHUNK_SIZE = 4096

INPUT_DIR = '../data/'
OUTPUT_DIR = '../outputs/'
# Create a directory to store the inference results
if not os.path.isdir(OUTPUT_DIR):
    os.mkdir(OUTPUT_DIR)


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
        data = self.recv(self._remaining_bytes)
        self._buffer += data
        self._remaining_bytes -= len(data)

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

        # yolov3 model inference with imageData
        result = inference(model, self._image_data)

        # send inference result from 200DK to PC
        data = pickle.dumps(result)

        # capture the length of the data portion of the message
        length = struct.pack('<I', len(data))

        # for the message transmission
        self._frame_data = b''.join([length, data])
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
            sock, addr = pair
            print('Incoming connection from %s' % repr(addr))

            # when a connection is attempted, delegate image receival to the ImageClient
            self.handler = ImageClient(sock, addr)
            self._image_data = self.handler.get_img()


def preprocess_cv2(bgr_img):
    """
    Preprocess cv2 bgr image for yolov3 input
    :param bgr_img: image with BGR format
    :return: processed image, MODEL_WIDTH, MODEL_HEIGHT
    """

    shape = bgr_img.shape[:2]  # [height, width]

    net_h = MODEL_HEIGHT
    net_w = MODEL_WIDTH
    scale = min(float(net_h) / float(shape[0]), float(net_w) / float(shape[1]))
    new_h = int(shape[0] * scale)
    new_w = int(shape[1] * scale)

    dw = (net_w - new_w) / 2
    dh = (net_h - new_h) / 2

    # yolov3 isobi scaling
    img = cv.resize(bgr_img, (new_w, new_h), interpolation=cv.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv.copyMakeBorder(img, top, bottom, left, right, cv.BORDER_CONSTANT, value=(0, 0, 0))

    return img, shape[1], shape[0]


def overlap(x1, x2, x3, x4):
    """
    Calculate the width/height of overlap area between bbox1 and bbox2
    :param x1: x_min or y_min of bbox1
    :param x2: x_max or y_max of bbox1
    :param x3: x_min or y_min of bbox2
    :param x4: x_max or y_max of bbox2
    :return:
    """

    left = max(x1, x3)
    right = min(x2, x4)

    return right - left


def cal_iou(box, truth):
    """
    Calculate IOU between box and truth
    :param box: bounding box: [x_min, y_min, x_max, y_max]
    :param truth: bounding box: [x_min, y_min, x_max, y_max]
    :return: IOU between box and truth
    """

    w = overlap(box[0], box[2], truth[0], truth[2])
    h = overlap(box[1], box[3], truth[1], truth[3])

    if w <= 0 or h <= 0:
        return 0

    inter_area = w * h
    union_area = (box[2] - box[0]) * (box[3] - box[1]) + (truth[2] - truth[0]) * (truth[3] - truth[1]) - inter_area

    return inter_area * 1.0 / union_area


def apply_nms(all_boxes, thres):
    """
    Hard nms for yolov3 output boxes's postprocess
    :param all_boxes: yolov3 output boxes
    :param thres: nms threshhold
    :return: nms result
    """

    res = []

    for cls in range(CLASS_NUM):
        cls_bboxes = all_boxes[cls]
        sorted_boxes = sorted(cls_bboxes, key=lambda d: d[5])[::-1]

        p = dict()
        for i in range(len(sorted_boxes)):
            if i in p:
                continue

            truth = sorted_boxes[i]
            for j in range(i + 1, len(sorted_boxes)):
                if j in p:
                    continue
                box = sorted_boxes[j]
                iou = cal_iou(box, truth)
                if iou >= thres:
                    p[j] = 1

        for i in range(len(sorted_boxes)):
            if i not in p:
                res.append(sorted_boxes[i])

    return res


def decode(conv_output, img_w, img_h):
    """
    Decode 3 output feature maps to object detection result
    :param conv_output: 3 output feature maps
    :param img_w: original image width
    :param img_h: original image height
    :return: object detection result
    """

    h, w, _ = conv_output.shape
    pred = conv_output.reshape((h * w, 3, 5 + CLASS_NUM))
    resize_ratio = min(MODEL_WIDTH / img_w, MODEL_HEIGHT / img_h)
    dw = (MODEL_WIDTH - resize_ratio * img_w) / 2
    dh = (MODEL_HEIGHT - resize_ratio * img_h) / 2

    bbox = np.zeros((h * w, 3, 4))
    bbox[..., 0] = np.maximum((pred[..., 0] - pred[..., 2] / 2.0 - dw) / resize_ratio, 0)  # x_min
    bbox[..., 1] = np.maximum((pred[..., 1] - pred[..., 3] / 2.0 - dh) / resize_ratio, 0)  # y_min
    bbox[..., 2] = np.minimum((pred[..., 0] + pred[..., 2] / 2.0 - dw) / resize_ratio, img_w)  # x_max
    bbox[..., 3] = np.minimum((pred[..., 1] + pred[..., 3] / 2.0 - dh) / resize_ratio, img_h)  # y_max

    pred[..., :4] = bbox
    pred = pred.reshape((-1, 5 + CLASS_NUM))
    pred[:, 4] = pred[:, 4] * pred[:, 5:].max(1)
    pred = pred[pred[:, 4] >= CONF_THRESHOLD]
    pred[:, 5] = np.argmax(pred[:, 5:], axis=-1)

    all_boxes = [[] for ix in range(CLASS_NUM)]
    for ix in range(pred.shape[0]):
        box = [int(pred[ix, iy]) for iy in range(4)]
        box.append(int(pred[ix, 5]))
        box.append(pred[ix, 4])
        all_boxes[box[4] - 1].append(box)

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

    result_return = dict()
    all_boxes = [[] for ix in range(CLASS_NUM)]
    for ix in range(3):
        pred = infer_output[ix].reshape((MODEL_HEIGHT // STRIDE_LIST[ix], MODEL_WIDTH // STRIDE_LIST[ix], NUM_CHANNEL))
        boxes = decode(pred, img_w, img_h)
        all_boxes = [all_boxes[iy] + boxes[iy] for iy in range(CLASS_NUM)]
    res = apply_nms(all_boxes, IOU_THRESHOLD)
    if not res:
        result_return['detection_classes'] = []
        result_return['detection_boxes'] = []
        result_return['detection_scores'] = []
    else:
        new_res = np.array(res)
        picked_boxes = new_res[:, 0:4]
        picked_boxes = picked_boxes[:, [1, 0, 3, 2]]
        picked_classes = convert_labels(new_res[:, 4])
        picked_score = new_res[:, 5]
        result_return['detection_classes'] = picked_classes
        result_return['detection_boxes'] = picked_boxes.tolist()
        result_return['detection_scores'] = picked_score.tolist()

    return result_return


def inference(model, bgr_img):
    """
    Yolov3 model inference pipeline
    :param model: yolov3 om model
    :param bgr_img: opencv bgr image
    :return: yolov3 detection result
    """

    if bgr_img.shape[0] > 0:
        t_pre = 0
        t_for = 0
        t_post = 0

        # preprocess image
        t1 = time.time()
        data, w, h = preprocess_cv2(bgr_img)
        t2 = time.time()
        t_pre += (t2 - t1)

        # model inference
        result_list = model.execute([data, ])
        t3 = time.time()
        t_for += (t3 - t2)

        # post process
        result_return = post_process(result_list, w, h)
        t4 = time.time()
        t_post += (t4 - t3)

        print("*" * 40)
        print("result = ", result_return)
        print("preprocess cost：", t2 - t1)
        print("forward cost：", t3 - t2)
        print("proprocess cost：", t4 - t3)
        print("FPS:", 1 / (t4 - t1))
        print("*" * 40)

        return result_return
    else:
        return dict()


if __name__ == "__main__":
    # acl resource init
    acl_resource = AclResource()
    acl_resource.init()

    # load om model
    model = Model(MODEL_PATH)

    # model inference
    input_dir = os.listdir(INPUT_DIR)
    print("input_dir = ", input_dir)

    for pic in input_dir:
        # read image
        image_path = os.path.join(INPUT_DIR, pic)
        bgr_img = cv.imread(image_path)
        # preprocess image
        data, w, h = preprocess_cv2(bgr_img)
        # model inference
        result_list = model.execute([data, ])
        # post process
        result_return = post_process(result_list, w, h)

        print("result = ", result_return)
        for i in range(len(result_return['detection_classes'])):
            box = result_return['detection_boxes'][i]
            class_name = result_return['detection_classes'][i]
            confidence = result_return['detection_scores'][i]
            cv.rectangle(bgr_img, (int(box[1]), int(box[0])), (int(box[3]), int(box[2])), COLORS[i % 6], 2)
            p3 = (max(int(box[1]), 15), max(int(box[0]), 15))
            out_label = class_name
            cv.putText(bgr_img, out_label, p3, cv.FONT_ITALIC, 0.6, COLORS[i % 6], 1)
        output_file = os.path.join(OUTPUT_DIR, pic)
        print("output:%s" % output_file)
        cv.imwrite(output_file, bgr_img)

    print("Execute end")
