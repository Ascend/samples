# coding=utf-8
#
# Copyright 2022 Huawei Technologies Co., Ltd
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

import argparse
import base64
import json
import os
import time
import yaml

import numpy as np
import cv2
from albumentations.augmentations import transforms

import MxpiDataType_pb2 as mxpi_data
from StreamManagerApi import InProtobufVector
from StreamManagerApi import MxProtobufIn
from StreamManagerApi import StreamManagerApi


def save_color_png(img, msk, color):
    msk = msk + 0.5
    msk = cv2.resize(msk, (img.shape[1], img.shape[0]))
    msk = np.array(msk, np.uint8)
    contours, _ = cv2.findContours(
        msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if color == 'red':
        cv2.drawContours(img, contours, -1, (0, 0, 255), 1)
        img[..., 2] = np.where(msk == 1, 255, img[..., 2])
    elif color == 'green':
        cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
        img[..., 1] = np.where(msk == 1, 255, img[..., 1])
    elif color == 'blue':
        cv2.drawContours(img, contours, -1, (255, 0, 0), 1)
        img[..., 0] = np.where(msk == 1, 255, img[..., 0])

    return img


def check_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir, exist_ok=True)


class SDKInferWrapper:
    def __init__(self): # 完成初始化
        self._stream_name = None
        self._stream_mgr_api = StreamManagerApi()

        if self._stream_mgr_api.InitManager() != 0:
            raise RuntimeError("Failed to init stream manager.")

    def load_pipeline(self, pipeline_path):
        with open(pipeline_path, 'r') as f:
            pipeline = json.load(f)

        self._stream_name = list(pipeline.keys())[0].encode() # 'unet++_pytorch'
        if self._stream_mgr_api.CreateMultipleStreams(
                json.dumps(pipeline).encode()) != 0:
            raise RuntimeError("Failed to create stream.")

    def do_infer(self, image):
        tensor_pkg_list = mxpi_data.MxpiTensorPackageList()
        tensor_pkg = tensor_pkg_list.tensorPackageVec.add()
        tensor_vec = tensor_pkg.tensorVec.add()
        tensor_vec.deviceId = 0
        tensor_vec.memType = 0

        for dim in [1, *image.shape]:
            tensor_vec.tensorShape.append(dim) # tensorshape属性为[1,3,96,96]

        input_data = image.tobytes()
        tensor_vec.dataStr = input_data
        tensor_vec.tensorDataSize = len(input_data)

        protobuf_vec = InProtobufVector()
        protobuf = MxProtobufIn()
        protobuf.key = b'appsrc0'
        protobuf.type = b'MxTools.MxpiTensorPackageList'
        protobuf.protobuf = tensor_pkg_list.SerializeToString()
        protobuf_vec.push_back(protobuf)

        unique_id = self._stream_mgr_api.SendProtobuf(
            self._stream_name, 0, protobuf_vec)

        if unique_id < 0:
            raise RuntimeError("Failed to send data to stream.")

        infer_result = self._stream_mgr_api.GetResult(
            self._stream_name, unique_id)

        if infer_result.errorCode != 0:
            raise RuntimeError(
                f"GetResult error. errorCode={infer_result.errorCode}, "
                f"errorMsg={infer_result.data.decode()}")
        return infer_result


def _parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--infer_quant", required=False, default=False, type=bool)
    parser.add_argument("--dataset_dir", default='../../sample_data',
                        type=str, help="path of dataset directory")
    parser.add_argument("--pipeline", default='../config/nested_unet.pipeline',
                        type=str, help="path of pipeline file")
    # parser.add_argument("--output_dir", default="./infer_result_quant", # Revise
                        # type=str, help="path of output directory")
    return parser.parse_args()


def _parse_output_data(output_data):
    infer_result_data = json.loads(output_data.data.decode())
    content = json.loads(infer_result_data['metaData'][0]['content'])
    tensor_vec = content['tensorPackageVec'][0]['tensorVec'][0]
    data_str = tensor_vec['dataStr']
    tensor_shape = tensor_vec['tensorShape']
    infer_array = np.frombuffer(base64.b64decode(data_str), dtype=np.float32)
    return infer_array.reshape(tensor_shape)


def sigmoid(x):
    y = x.copy()
    y[x >= 0] = 1.0 / (1 + np.exp(-x[x >= 0]))
    y[x < 0] = np.exp(x[x < 0]) / (1 + np.exp(x[x < 0]))
    return y


def iou_score(output, target):
    smooth = 1e-5

    output_ = output > 0.5
    target_ = target > 0.5
    intersection = (output_ & target_).sum()
    union = (output_ | target_).sum()

    return (intersection + smooth) / (union + smooth)


def check_dir(dir):
    if not os.path.exists(dir):
        os.makedirs(dir, exist_ok=True)


def main():
    args = _parser_args()
    infer_quant = args.infer_quant
    output_folder = None
    if infer_quant:
        output_folder = "./infer_result_quant"
    else:
        output_folder = "./infer_result"   
    print("output_folder is", output_folder)     
    sdk_infer = SDKInferWrapper()
    sdk_infer.load_pipeline(args.pipeline)

    f = open('../../config.yml', encoding='utf-8')
    config = yaml.load(f.read(), Loader=yaml.FullLoader)
    num_class = config['num_classes']

    count = 0
    iou_sum = 0
    image_dir = os.path.join(args.dataset_dir, 'images')
    mask_dir = os.path.join(args.dataset_dir, 'masks')
    img_ids = os.listdir(image_dir)

    pre_t = []
    infer_t = []
    post_t = []
    total_t = []

    for _ in range(10):
        for image_id in img_ids:
            # read img
            img_bgr = cv2.imread(os.path.join(image_dir, image_id))

            # read masks (labels)
            mask = []
            for i in range(num_class):
                curr_mask_dir = os.path.join(mask_dir, str(i))
                file_name = image_id.split('.')[0]
                curr_mask = cv2.imread(os.path.join(curr_mask_dir, file_name+'.png'), cv2.IMREAD_GRAYSCALE)[..., None]
                curr_mask = curr_mask.astype('float32') / 255
                curr_mask = curr_mask.transpose(2, 0, 1)
                mask.append(curr_mask)

            if img_bgr is None or mask is None:
                raise RuntimeError(f"Failed to get image by id {image_id}")

            t0 = time.time()
            # preprocess
            image = cv2.resize(img_bgr, (96, 96))
            nor = transforms.Normalize()
            image = nor.apply(image)
            image = image.astype('float32') / 255
            image = image.transpose(2, 0, 1)

            # infer
            t1 = time.time()
            output_data = sdk_infer.do_infer(image)
            t2 = time.time()
            
            output_tensor = _parse_output_data(output_data)
            # cal_iou
            os.makedirs(output_folder, exist_ok=True)
            color_list = ['red', 'green', 'blue']
            for i in range(num_class):

                tensor = sigmoid(output_tensor[0][i])
                iou = iou_score(tensor, mask[i])
                iou_sum += iou
                count += 1

                # get color msk
                img_bgr = save_color_png(img_bgr, tensor, color_list[i])
            cv2.imwrite(os.path.join(output_folder, image_id), img_bgr)

            t3 = time.time()

            pre_t.append(t1-t0)
            infer_t.append(t2-t1)
            post_t.append(t3-t2)
            total_t.append(t3-t0)

    print('=='*30)
    print(f"The Mean IOU is {iou_sum/count}")
    # print("pre_t is", np.array(pre_t))
    print('pre_t\t{} ms'.format(np.mean(np.array(pre_t))*1000))
    # print("infer_t is", np.array(infer_t))
    print('infer_t\t{} ms'.format(np.mean(np.array(infer_t))*1000))
    print('post_t\t{} ms'.format(np.mean(np.array(post_t))*1000))
    print('total_t\t{} ms'.format(np.mean(np.array(total_t))*1000))


if __name__ == "__main__":
    main()
