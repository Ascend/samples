# Copyright 2021 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================

import os
import cv2
import numpy as np

Colors_1 = [
        230, 150, 87, 34, 65, 92, 139, 210, 60, 189, 240, 21,
        40, 56, 77, 101, 168, 179, 199, 222]
Colors_2 = [
        46, 235, 187, 234, 165, 222, 188, 0, 153, 43, 70, 221,
        184, 177, 217, 101, 32, 209, 199, 32]
Colors_3 = [
        166, 50, 287, 134, 55, 192, 39, 210, 90, 109, 170, 161,
        40, 156, 177, 49, 48, 79, 89, 122]

def draw_label(img_file, mask_pred, label_img_dir):
    segments = list()
    img = cv2.imread(img_file)
    #get img shape
    out_img = cv2.resize(img, (1280,720))
    img_h, img_w = out_img.shape[:2]
    mask_pred = mask_pred > 0.5
    mask_pred = mask_pred.astype("uint8")
    for i in range(1,21):
        #判断是否是零矩阵
        if np.any(mask_pred[i][...] >= 1):
            mask_pred_ = mask_pred[i, :, :]

            im_mask = cv2.resize(mask_pred_, (img_w, img_h))
            test_mask = cv2.cvtColor(im_mask, cv2.COLOR_GRAY2BGR)
            dilate_mask_pic = "./mask/pic" + str(i) +".jpg" 
            cv2.imwrite(dilate_mask_pic, im_mask)
            blue, green, red = cv2.split(test_mask)  # get single color
            blue = blue * Colors_1[i]
            green = green * Colors_2[i]
            red = red * Colors_3[i]
            resupper_blue = cv2.merge([blue, green, red])
            out_img = cv2.addWeighted(out_img, 1, resupper_blue, 1, 0)

    _, file_name = os.path.split(img_file)
    label_path = os.path.join(label_img_dir, file_name)
    cv2.imwrite(label_path, out_img)

