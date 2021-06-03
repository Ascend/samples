"""Copyright 2020 Huawei Technologies Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
You may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License."""

import numpy as np
import copy
import math


class WHENet(object):
    """WHENet"""
    def __init__(self, camera_width, camera_height, whenet_model):
        self.whenet = whenet_model
        self.camera_height = camera_height
        self.camera_width = camera_width

    def inference(self, nparryList, box_width, box_height):
        """
        WHENet preprocessing, inference and postprocessing
        Args: 
            nparryList: result from YOLO V3, which is detected head area
            box_width: width of the detected area
            box_height: height of the detected area
        Returns:
            return yaw pitch roll value values in numpy format
        """
        resultList_whenet = self.whenet.execute([nparryList])
        # postprocessing: convert model output to yaw pitch roll value
        yaw, pitch, roll = self.whenet_angle(resultList_whenet)
        print('Yaw, pitch, roll angles: ', yaw, pitch, roll)
        # obtain coordinate points from head pose angles for plotting
        return self.whenet_draw(yaw, pitch, roll, 
                                tdx=box_width, tdy=box_height, size=200)

    def softmax(self, x):
        """softmax"""
        x -= np.max(x, axis=1, keepdims=True)
        a = np.exp(x)
        b = np.sum(np.exp(x), axis=1, keepdims=True)
        return a / b

    def whenet_draw(self, yaw, pitch, roll, tdx=None, tdy=None, size=200):
        """
        Plot lines based on yaw pitch roll values

        Args:
            yaw, pitch, roll: values of angles
            tdx, tdy: center of detected head area

        Returns:
            graph: locations of three lines
        """
        # taken from hopenet
        pitch = pitch * np.pi / 180
        yaw = -(yaw * np.pi / 180)
        roll = roll * np.pi / 180

        tdx = tdx
        tdy = tdy

        # X-Axis pointing to right. drawn in red
        x1 = size * (math.cos(yaw) * math.cos(roll)) + tdx
        y1 = size * (math.cos(pitch) * math.sin(roll) + math.cos(roll)
                     * math.sin(pitch) * math.sin(yaw)) + tdy

        # Y-Axis | drawn in green
        x2 = size * (-math.cos(yaw) * math.sin(roll)) + tdx
        y2 = size * (math.cos(pitch) * math.cos(roll) - math.sin(pitch)
                     * math.sin(yaw) * math.sin(roll)) + tdy

        # Z-Axis (out of the screen) drawn in blue
        x3 = size * (math.sin(yaw)) + tdx
        y3 = size * (-math.cos(yaw) * math.sin(pitch)) + tdy
        
        
        return {
            "yaw_x": x1,
            "yaw_y": y1, 
            "pitch_x": x2, 
            "pitch_y": y2, 
            "roll_x": x3, 
            "roll_y": y3
        }

    def whenet_angle(self, resultList_whenet):
        """
        Obtain yaw pitch roll value in degree based on the output of model

        Args:
            resultList_whenet: result of WHENet

        Returns:
            yaw_predicted, pitch_predicted, roll_predicted: yaw pitch roll values
        """
        yaw = resultList_whenet[0]
        yaw = np.reshape(yaw, (1, 120, 1, 1))
        yaw_out = np.transpose(yaw, (0, 2, 3, 1)).copy()
        yaw_out = yaw_out.squeeze()
        yaw_out = np.expand_dims(yaw_out, axis=0)

        pitch = resultList_whenet[1]
        pitch = np.reshape(pitch, (1, 66, 1, 1))
        pitch_out = np.transpose(pitch, (0, 2, 3, 1)).copy()
        pitch_out = pitch_out.squeeze()
        pitch_out = np.expand_dims(pitch_out, axis=0)

        roll = resultList_whenet[2]
        roll = np.reshape(roll, (1, 66, 1, 1))
        roll_out = np.transpose(roll, (0, 2, 3, 1)).copy()
        roll_out = roll_out.squeeze()
        roll_out = np.expand_dims(roll_out, axis=0)

        yaw_predicted = self.softmax(yaw_out)
        pitch_predicted = self.softmax(pitch_out)
        roll_predicted = self.softmax(roll_out)

        idx_tensor_yaw = [idx for idx in range(120)]
        idx_tensor_yaw = np.array(idx_tensor_yaw, dtype=np.float32)

        idx_tensor = [idx for idx in range(66)]
        idx_tensor = np.array(idx_tensor, dtype=np.float32)

        yaw_predicted = np.sum(
            yaw_predicted * idx_tensor_yaw, axis=1) * 3 - 180
        pitch_predicted = np.sum(pitch_predicted * idx_tensor, axis=1) * 3 - 99
        roll_predicted = np.sum(roll_predicted * idx_tensor, axis=1) * 3 - 99

        return np.array(yaw_predicted), np.array(pitch_predicted), np.array(roll_predicted)
