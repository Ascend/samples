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
import cv2

sys.path.append("../../../common")
from acllite_resource import AclLiteResource
from acllite_model import AclLiteModel
from acllite_image import AclLiteImage

from ChessStatusPerception.ChessStatusPerception import ChessStatusPerception

MODEL_PATH = "../model/chess_ckpt_0804_vgg_99_0.om"
INPUT_DIR = '../data/'


def test_200dk():
    """
    Test the output of ChessStatusPerception module.
    """
    acl_resource = AclLiteResource()
    acl_resource.init()
    perception = ChessStatusPerception(MODEL_PATH)
    perception.Init()

    # load input
    print("input_dir = ", INPUT_DIR)
    img = os.path.join(INPUT_DIR, "test.jpg")
    image = cv2.imread(img)
    # execute the chessboard status perception function
    chessStatus, chessStatus_real = perception.Process(image)

    return chessStatus


def list_equal(list1, list2):
    """
    Used for judge if 2 lists contains the same elements.
    """
    flag = True
    for item1 in list1:
        if item1 not in list2:
            flag = False

    for item2 in list2:
        if item2 not in list1:
            flag = False

    return flag


if __name__ == '__main__':
    output = test_200dk()
    truth = [[5, 6, 13], [1, 5, 0], [5, 5, 9], [7, 3, 10], [3, 2, 5], [8, 4, 7], [4, 4, 6], [6, 2, 13]]
    ans = list_equal(truth, output)
    assert ans == True


