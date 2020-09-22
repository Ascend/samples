# Copyright 2020 Huawei Technologies Co., Ltd
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
import numpy as np
import os
from PIL import Image

def process(input_path):
    im = Image.open(input_path)
    im = im.resize((256,256))
    # hwc
    img = np.array(im)
    height = img.shape[0]
    width = img.shape[1]
    h_off = int((height-224)/2)
    w_off = int((width-224)/2)
    crop_img = img[h_off:height-h_off, w_off:width-w_off, :]
    # rgb to bgr
    img = crop_img[:,:,::-1]
    shape = img.shape
    img = img.astype("float16")
    img[:,:,0] -= 104
    img[:,:,1] -= 117
    img[:,:,2] -= 123
    img = img.reshape([1] + list(shape))
    result = img.transpose([0, 3, 1, 2])

    outputName = input_path.split('.')[0] + ".bin"
    result.tofile(outputName)

if __name__ == "__main__":
    images = os.listdir(r'./')
    for image_name in images:
        if not image_name.endswith("jpg"):
            continue

        print("start to process image {}....".format(image_name))
        process(image_name)
