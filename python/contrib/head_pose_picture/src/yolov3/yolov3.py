"""Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License."""

import yolov3.yolov3_postprocessing as postprocessing
import numpy as np
import cv2
import os


class YOLOV3(object):
    """YOLOv3"""
    def __init__(self, camera_height, camera_width, yolo_model):
        # load YOLO model
        self.yolo_v3 = yolo_model
        # parameters for preprocessing
        self.ih, self.iw = (camera_height, camera_width)
        self.h, self.w = (416, 416)
        self.scale = min(self.w / self.iw, self.h / self.ih)
        self.nw = int(self.iw * self.scale)
        self.nh = int(self.ih * self.scale)

        # parameters for postprocessing
        self.image_shape = [camera_height, camera_width]
        self.model_shape = [self.h, self.w]
        self.num_classes = 1
        self.anchors = self.get_anchors()

    def get_anchors(self):
        """return anchors

        Returns:
            [ndarray]: anchors array
        """
        SRC_PATH = os.path.realpath(__file__).rsplit("/", 1)[0]
        anchors_path = os.path.join(SRC_PATH, './yolo_anchors.txt')
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def inference(self, img):
        """
        Args:
            img ([ndarray]): image (416, 416, 3)
        """
        # preprocessing: resize and paste input image to a new image with size 416*416
        img = np.array(img, dtype='float32')
        img_resize = cv2.resize(img, (self.nw, self.nh),
                                interpolation=cv2.INTER_CUBIC)
        img_new = np.ones((416, 416, 3), np.float32) * 128
        img_new[(self.h - self.nh) // 2: ((self.h - self.nh) // 2 + self.nh),
                (self.w - self.nw) // 2: (self.w - self.nw) // 2 + self.nw, :] = img_resize[:, :, :]
        img_new = img_new / 255.
        # inference
        resultList = self.yolo_v3.execute([img_new])
        out_list = [resultList[0], resultList[1], resultList[2]]
        # convert yolo output to box axis and score
        box_axis, box_score = postprocessing.yolo_eval(
            out_list, self.anchors, self.num_classes, self.image_shape)
        # get the crop image and corresponding width/heigh info for WHENet
        nparryList, boxList = postprocessing.get_box_img(img, box_axis)

        return nparryList, boxList
