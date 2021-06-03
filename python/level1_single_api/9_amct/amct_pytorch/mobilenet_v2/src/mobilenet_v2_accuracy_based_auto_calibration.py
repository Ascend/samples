"""
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
"""


import os
import argparse

import cv2
import numpy as np
import torch
import torchvision.models as models

import amct_pytorch as amct
from amct_pytorch.common.auto_calibration import AutoCalibrationEvaluatorBase


PATH = os.path.realpath('./')
IMG_DIR = os.path.join(PATH, './data/images')
LABLE_FILE = os.path.join(IMG_DIR, 'image_label.txt')

PARSER = argparse.ArgumentParser(description='amct_pytorch mobilenet v2 accuracy based auto calibration sample.')
ARGS = PARSER.parse_args()

TMP = os.path.join(PATH, 'tmp')

def get_labels_from_txt(label_file):
    """Read all images' name and label from label_file"""
    images = []
    labels = []
    with open(label_file, 'r') as mfile:
        lines = mfile.readlines()
        for line in lines:
            images.append(line.split(' ')[0])
            labels.append(int(line.split(' ')[1]))
    return images, labels


def prepare_image_input(
    images, height=256, width=256, crop_size=224):
    """Read image files to blobs [batch_size, 3, 224, 224]"""
    mean=[0.485, 0.456, 0.406]
    std=[0.229, 0.224, 0.225]
    input_tensor = np.zeros((len(images), 3, crop_size, crop_size), np.float32)

    imgs = np.zeros((len(images), 3, height, width), np.float32)
    for index, im_file in enumerate(images):
        im_data = cv2.imread(im_file)
        im_data = cv2.resize(im_data, (256, 256), interpolation=cv2.INTER_CUBIC)
        cv2.cvtColor(im_data, cv2.COLOR_BGR2RGB)

        imgs[index, :, :, :] = im_data.transpose(2, 0, 1).astype(np.float32)

    h_off = int((height - crop_size) / 2)
    w_off = int((width - crop_size) / 2)
    input_tensor = imgs[:, :, h_off: (h_off + crop_size), w_off: (w_off + crop_size)]
    # trans uint8 image data to float
    input_tensor /= 255
    # do channel-wise reduce mean value
    for channel in range(input_tensor.shape[1]):
        input_tensor[:, channel, :, :] -= mean[channel]
    # do channel-wise divide std
    for channel in range(input_tensor.shape[1]):
        input_tensor[:, channel, :, :] /= std[channel]

    return input_tensor


def img_postprocess(probs, labels):
    """Do image post-process"""
    # calculate top1 and top5 accuracy
    top1_get = 0
    top5_get = 0
    prob_size = probs.shape[1]
    for index, label in enumerate(labels):
        top5_record = (probs[index, :].argsort())[prob_size - 5: prob_size]
        if label == top5_record[-1]:
            top1_get += 1
            top5_get += 1
        elif label in top5_record:
            top5_get += 1
    return float(top1_get) / len(labels), float(top5_get) / len(labels)


def model_forward(model, batch_size=1, iterations=160):
    """forward"""
    model.eval()
    images, labels = get_labels_from_txt(LABLE_FILE)
    images = [os.path.join(IMG_DIR, image) for image in images]
    top1_total = 0
    top5_total = 0
    for i in range(iterations):
        input_batch = prepare_image_input(images[i * batch_size: (i + 1) * batch_size])
        input_batch = torch.tensor(input_batch)
        # move the input and model to GPU for speed if available
        if torch.cuda.is_available():
            input_batch = input_batch.to('cuda')
            model.to('cuda')

        with torch.no_grad():
            output = model(input_batch)
        top1, top5 = img_postprocess(output, labels[i * batch_size: (i + 1) * batch_size])
        top1_total += top1
        top5_total += top5
        print('****************iteration:{}*****************'.format(i))
        print('top1_acc:{}'.format(top1))
        print('top5_acc:{}'.format(top5))
    print('******top1:{}'.format(top1_total / iterations))
    print('******top5:{}'.format(top5_total / iterations))
    return top1_total / iterations, top5_total / iterations


# You need to implement the AutoCalibrationEvaluator's calibration(), evaluate() and metric_eval() funcs
class AutoCalibrationEvaluator(AutoCalibrationEvaluatorBase):
    """ subclass of AutoCalibrationEvaluatorBase"""
    def __init__(self, target_loss, batch_num):
        super(AutoCalibrationEvaluator, self).__init__()
        self.target_loss = target_loss
        self.batch_num = batch_num

    def calibration(self, model):
        """ implement the calibration function of AutoCalibrationEvaluatorBase
            calibration() need to finish the calibration inference procedure
            so the inference batch num need to >= the batch_num pass to create_quant_config
        """
        model_forward(model=model, batch_size=32, iterations=self.batch_num)

    def evaluate(self, model):
        """ implement the evaluate function of AutoCalibrationEvaluatorBase
            params: model in torch.nn.module 
            return: the accuracy of input model on the eval dataset, or other metric which
                    can describe the 'accuracy' of model
        """
        top1, _ = model_forward(model=model, batch_size=32, iterations=5)
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        return top1

    def metric_eval(self, original_metric, new_metric):
        """ implement the metric_eval function of AutoCalibrationEvaluatorBase
            params: original_metric: the returned accuracy of evaluate() on non quantized model
                    new_metric: the returned accuracy of evaluate() on fake quant model
            return:
                   [0]: whether the accuracy loss between non quantized model and fake quant model
                        can satisfy the requirement
                   [1]: the accuracy loss between non quantized model and fake quant model
        """
        loss = original_metric - new_metric
        if loss * 100 < self.target_loss:
            return True, loss
        return False, loss


def main():
    """ the main procedure of auto calibration"""
    model = models.mobilenet_v2(pretrained=True)

    model.eval()
    args_shape = [(1, 3, 224, 224)]
    input_data = tuple([torch.randn(arg_shape) for arg_shape in args_shape])
    if torch.cuda.is_available():
        input_data = tuple([data.to('cuda') for data in input_data])
        model.to('cuda')

    # 1. step1 create quant config json file
    config_json_file = os.path.join(TMP, 'config.json')
    skip_layers = []
    batch_num = 2
    amct.create_quant_config(
        config_json_file,
        model,
        input_data,
        skip_layers,
        batch_num
    )

    # 2. step2 construct the instance of AutoCalibrationEvaluator
    evaluator = AutoCalibrationEvaluator(target_loss=0.5, batch_num=batch_num)

    # 3. step3 using the accuracy_based_auto_calibration to quantized the model
    record_file = os.path.join(TMP, 'scale_offset_record.txt')
    result_path = os.path.join(PATH, 'result/mobilenet_v2')
    amct.accuracy_based_auto_calibration(
        model=model,
        model_evaluator=evaluator,
        config_file=config_json_file,
        record_file=record_file,
        save_dir=result_path,
        input_data=input_data,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        },
        strategy='BinarySearch',
        sensitivity='CosineSimilarity'
    )


if __name__ == "__main__":
    main()
