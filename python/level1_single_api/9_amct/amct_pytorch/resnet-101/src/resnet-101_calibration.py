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

import torch # pylint: disable=E0401
from PIL import Image # pylint: disable=E0401
from torchvision import transforms # pylint: disable=E0401
import onnxruntime as ort # pylint: disable=E0401

import amct_pytorch as amct # pylint: disable=E0401
from resnet import resnet101 # pylint: disable=E0401, C0415


PATH = os.path.realpath('./')
IMG_DIR = os.path.join(PATH, 'data/images')
LABEL_FILE = os.path.join(IMG_DIR, 'image_label.txt')

PARSER = argparse.ArgumentParser(description='whether use nuq')
PARSER.add_argument('--nuq', dest='nuq', action='store_true', help='whether use nuq')
ARGS = PARSER.parse_args()

if ARGS.nuq:
    OUTPUTS = os.path.join(PATH, 'outputs/nuq')
else:
    OUTPUTS = os.path.join(PATH, 'outputs/calibration')

TMP = os.path.join(OUTPUTS, 'tmp')


def get_labels_from_txt(label_file):
    """Read all images' name and label from label_file"""
    images = []
    labels = []
    with open(label_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            images.append(line.split(' ')[0])
            labels.append(int(line.split(' ')[1]))
    return images, labels


def prepare_image_input(images):
    """Read all images"""
    input_tensor = torch.zeros(len(images), 3, 224, 224) # pylint: disable=E1101
    preprocess = transforms.Compose(
        [transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
    for index, image in enumerate(images):
        input_image = Image.open(image).convert('RGB')
        input_tensor[index, ...] = preprocess(input_image)
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


def model_forward(model, batch_size, iterations):
    """Do pytorch model forward"""
    images, labels = get_labels_from_txt(LABEL_FILE)
    images = [os.path.join(IMG_DIR, image) for image in images]
    top1_total = 0
    top5_total = 0
    for i in range(iterations):
        input_batch = prepare_image_input(images[i * batch_size: (i + 1) * batch_size])
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
    print('******final top1:{}'.format(top1_total / iterations))
    print('******final top5:{}'.format(top5_total / iterations))
    return top1_total / iterations, top5_total / iterations


def onnx_forward(onnx_model, batch_size, iterations):
    """Do onnx model forward"""
    ort_session = ort.InferenceSession(onnx_model)

    images, labels = get_labels_from_txt(LABEL_FILE)
    images = [os.path.join(IMG_DIR, image) for image in images]
    top1_total = 0
    top5_total = 0
    for i in range(iterations):
        input_batch = prepare_image_input(images[i * batch_size: (i + 1) * batch_size])
        output = ort_session.run(None, {'input': input_batch.numpy()})
        top1, top5 = img_postprocess(output[0], labels[i * batch_size: (i + 1) * batch_size])
        top1_total += top1
        top5_total += top5
        print('****************iteration:{}*****************'.format(i))
        print('top1_acc:{}'.format(top1))
        print('top5_acc:{}'.format(top5))
    print('******final top1:{}'.format(top1_total / iterations))
    print('******final top5:{}'.format(top5_total / iterations))
    return top1_total / iterations, top5_total / iterations


def main():
    """Sample main function"""
    model = resnet101(pretrained=True)

    model.eval()
    ori_top1, ori_top5 = model_forward(model, batch_size=32, iterations=5)

    # Quantize configurations
    args_shape = [(1, 3, 224, 224)]
    input_data = tuple([torch.randn(arg_shape) for arg_shape in args_shape]) # pylint: disable=E1101
    if torch.cuda.is_available():
        input_data = tuple([data.to('cuda') for data in input_data])
        model.to('cuda')
    config_json_file = os.path.join(TMP, 'config.json')
    skip_layers = []
    batch_num = 2

    if ARGS.nuq:
        config_defination = os.path.join(PATH, 'src/nuq_conf/nuq_quant.cfg')
        amct.create_quant_config(
            config_json_file, model, input_data, skip_layers, batch_num, config_defination=config_defination)
    else:
        amct.create_quant_config(config_json_file, model, input_data, skip_layers, batch_num)

    # Phase1: do conv+bn fusion, weights calibration and generate
    #         calibration model
    record_file = os.path.join(TMP, 'record.txt')
    modified_model = os.path.join(TMP, 'modified_model.onnx')
    calibration_model = amct.quantize_model(
        config_json_file, modified_model, record_file, model, input_data, input_names=['input'],
        output_names=['output'], dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}})

    # Phase2: do calibration
    model_forward(calibration_model, batch_size=32, iterations=batch_num)
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    # Phase3: save final model, one for onnx do fake quant test, one
    #         deploy model for ATC
    result_path = os.path.join(OUTPUTS, 'resnet-101')
    amct.save_model(modified_model, record_file, result_path)

    # Phase4: run fake_quant model test
    quant_top1, quant_top5 = onnx_forward(
        '%s_%s' % (result_path, 'fake_quant_model.onnx'), batch_size=32, iterations=5)
    print('[INFO] ResNet101 before quantize top1:{:>10} top5:{:>10}'.format(ori_top1, ori_top5))
    print('[INFO] ResNet101 after quantize  top1:{:>10} top5:{:>10}'.format(quant_top1, quant_top5))


if __name__ == '__main__':
    main()
