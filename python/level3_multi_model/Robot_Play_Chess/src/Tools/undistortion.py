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
import time
DIM = (640, 480)
K = np.array(
        [[695.0543330294714, 0.0, 329.0492875591497], [0.0, 689.0308454966914, 230.88016146078488], [0.0, 0.0, 1.0]])
D = np.array([[-0.07668666257315113], [-0.6252340521255029], [1.9684570672441248], [1.0351859060772481]])

capture = cv2.VideoCapture(2)


def undistort(img, K, D, DIM, scale=1, imshow=False):
    """
    param：
        img
        K,D,DIM inner param of camera
    return:
        image after undistortion
    """

    dim1 = img.shape[:2][::-1]
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1],  "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if dim1[0] != DIM[0]:
        img = cv2.resize(img, DIM, interpolation=cv2.INTER_AREA)
    Knew = K.copy()
    if scale:  # change fov
        Knew[(0,1), (0,1)] = scale * Knew[(0,1), (0,1)]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    if imshow:
        cv2.imshow("undistorted", undistorted_img)
    return undistorted_img


index = 0
while True:
    ret, frame = capture.read()
    # Our operations on the frame come here

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('p'):
        cv2.imwrite("./dis_img_0.jpg", frame)
        undis_img = undistort(frame, K, D, DIM, 1, False)
        cv2.imwrite("./undis_img_0.jpg", undis_img)
        index = index + 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
capture.release()
cv2.destroyAllWindows()
