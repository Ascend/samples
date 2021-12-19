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

import numpy as np
import cv2
from acllite_model import AclLiteModel
import constants as const

CLS = ['b_jiang','b_ju', 'b_ma', 'b_pao', 'b_shi', 'b_xiang', 'b_zu',
        'r_bing', 'r_ju', 'r_ma', 'r_pao', 'r_shi', 'r_shuai', 'r_xiang']

mapList = [0, 1, 2, 3, 4, 5, 6, 13, 8, 9, 10, 11, 7, 12]

class ChessStatusPerception:
    def __init__(self, model_path):
        self.classifier = Classify(model_path)

    def Init(self):
        self.classifier.init()

    def Process(self, image):
        # undistortion
        image = self.Undistortion(image)
        # get circles info
        circlesInfo = self.GetCirclesInfo(image)
        if not circlesInfo:
            print("check light please")
            return []
        # classification
        categoryList = self.Classification(image, circlesInfo)
        # 2 kinds of position information
        boardPos, boardPos_real = self.BoardCalib(image, circlesInfo)
        # package information
        # chessStatus [Chess0Info, Chess1Info, ...]; Chess0Info = [x_coordinate, y_coordinate, category]
        chessStatus = []
        for i in range(0, len(categoryList)):
            chessStatus.append([boardPos[i][0], boardPos[i][1], categoryList[i]])
            print("Position: ", [boardPos[i][0], boardPos[i][1], categoryList[i]])

        chessStatus_real = []
        for i in range(0, len(categoryList)):
            chessStatus_real.append([boardPos_real[i][0], boardPos_real[i][1], categoryList[i]])
            print("real_Position: ", [boardPos_real[i][0], boardPos_real[i][1], categoryList[i]])
        return chessStatus, chessStatus_real

    def Undistortion(self, image):
        """
        param:            
            img: raw image

        return:
            undistorted_img: image after undistortion
        """
        K = np.array([[695.0543330294714, 0.0, 329.0492875591497],
                           [0.0, 689.0308454966914, 230.88016146078488],
                           [0.0, 0.0, 1.0]])
        D = np.array([[-0.07668666257315113], [-0.6252340521255029], [1.9684570672441248], [1.0351859060772481]])
        DIM = (640, 480)
        scale = 1.0
        imshow = False
        img = image

        dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
        assert dim1[0] / dim1[1] == DIM[0] / DIM[
            1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        if dim1[0] != DIM[0]:
            img = cv2.resize(img, DIM, interpolation=cv2.INTER_AREA)
        Knew = K.copy()
        if scale:  # change fov
            Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        if imshow:
            cv2.imshow("undistorted", undistorted_img)
        return undistorted_img

    def GetCirclesInfo(self, image):
        """
        circlesInfo = [circle0Info, circle1Info...]
        circle0Info = [circleCenter, raius, leftUpCorner, rightBottomCorner]
        """
        print("GetCircleInfo Input Image Shape: ", image.shape)
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 1, 15,
                                param1=100, param2=30, minRadius=12, maxRadius=25)
        print("Detected Circles: ", circles)
        circles = circles.reshape(-1, 3)
        circles = np.uint16(np.around(circles))

        circlesInfo = []
        for i in circles.tolist():
            circleCenter = (i[1], i[0])
            raius = i[2]
            leftUpCorner = (circleCenter[0] - raius, circleCenter[1] - raius)
            rightBottomCorner = (circleCenter[0] + raius, circleCenter[1] + raius)
            circlesInfo.append([circleCenter, raius, leftUpCorner, rightBottomCorner])
        print("Detected Circles: ", len(circlesInfo))
        for circle in circlesInfo:
            print("Circle Info: ", circle)
        return circlesInfo

    def Classification(self, image, circlesInfo):
        """
        param:
            image: input image file
            circlesInfo: information of circles detected in the previous steps

        return:
            categoryList: classes of corresponding circles in the circlesInfo list
        """
        categoryList = []
        for circle in circlesInfo:
            circleImage = image[circle[2][0]:circle[3][0], circle[2][1]:circle[3][1]]
            print("circleImage Shape: ", circleImage.shape)
            label = mapList[self.classifier.process(circleImage)]
            categoryList.append(label)
        return categoryList

    def BoardCalib(self, image, circlesInfo):
        """
        mapping pixel coordinates to chessboard coordinates
        param:
            image: raw image
            circlesInfo: information of circles detected in the previous steps
        return:
            boardPos: ideal chessboard position information
            boardPos_real: real chessboard position information
        """
        corners = np.float32([[83, 98], [82, 465], [412, 98], [410, 467]])

        square = np.float32([[0, 800], [900, 800], [0, 0], [900, 0]])

        M = cv2.getPerspectiveTransform(corners, square)
        boardPos = []
        boardPos_real = []

        for circle in circlesInfo:

            [new_pos, real_pos] = self.PositionTrans(list(circle[0]), M)
            boardPos.append(new_pos)
            boardPos_real.append(real_pos)

        return boardPos, boardPos_real

    def PositionTrans(self, pos, cvt_mat_t):
        """
        Use Perspective transform.
        param:
            pos: original position coordinate
            cvt_mat_t: convert matrix t (perspective transform)
        return:
            [new_pos, real_pos]: 2 kinds of position information after perspective transform
        """
        u = pos[0]
        v = pos[1]

        x = (cvt_mat_t[0][0] * u + cvt_mat_t[0][1] * v + cvt_mat_t[0][2]) / (cvt_mat_t[2][0] * u + cvt_mat_t[2][1] * v + cvt_mat_t[2][2])
        y = (cvt_mat_t[1][0] * u + cvt_mat_t[1][1] * v + cvt_mat_t[1][2]) / (cvt_mat_t[2][0] * u + cvt_mat_t[2][1] * v + cvt_mat_t[2][2])

        x_map = round(x/100)
        y_map = round(y/100)
        new_pos = [x_map, y_map]
        real_pos = [int(x), int(y)]
        return [new_pos, real_pos]
    

class Classify(object):
    """
    Class for portrait segmentation
    """
    def __init__(self, model_path):
        self._model_path = model_path
        self._model = None
        
    def init(self):
        """
        Initialize
        """
        # Load model
        self._model = AclLiteModel(self._model_path)

        return const.SUCCESS

    def pre_process(self, img):
        """
        preprocess 
        """
        img = cv2.resize(img, (56, 56))
        img = img.astype(np.float32) / 255.0
        processed_img = np.expand_dims(img, axis=0)
        return processed_img  

    def inference(self, input_data):
        """
        model inference
        """
        return self._model.execute(input_data)

    def post_process(self, infer_output):
        """
        Post-processing, analysis of inference results
        """
        infer_result = infer_output[0]
        vals = infer_result.flatten()
        pre_index = vals.argsort()[-1]
        return pre_index 
        
    def process(self, input_image):
        """
        complete process
        """
        processed_img = self.pre_process(input_image)
        infer_output = self.inference(processed_img)
        result = self.post_process(infer_output)
        return result
