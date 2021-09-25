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

import cv2
import numpy as np

#
img = cv2.imread("undis_img_0.jpg")
cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
# =========================================================================

corners = np.float32([[72, 145], [63, 501], [385, 146], [386, 506]])

square = np.float32([[0, 800], [900, 800], [0, 0], [900, 0]])
# =========================================================================

pixel_for_test = [417, 203]
pixel_for_test_1 = [352, 440]
M = cv2.getPerspectiveTransform(corners, square)


def cvt_pos(pos: list, cvt_mat_t: list):
    """
    param:
        pos: original coord, [x, y]
    return:
        x_map, y_map: new coord after transform
    """
    u = pos[0]
    v = pos[1]
    x = (cvt_mat_t[0][0] * u + cvt_mat_t[0][1] * v + cvt_mat_t[0][2]) / (
                cvt_mat_t[2][0] * u + cvt_mat_t[2][1] * v + cvt_mat_t[2][2])
    y = (cvt_mat_t[1][0] * u + cvt_mat_t[1][1] * v + cvt_mat_t[1][2]) / (
                cvt_mat_t[2][0] * u + cvt_mat_t[2][1] * v + cvt_mat_t[2][2])
    x_map = round(x/10)
    y_map = round(y/10)
    return x_map, y_map


print(cvt_pos(pixel_for_test_1, M))
