import argparse

import numpy as np
import cv2 as cv

from mobilenet import MobileNet

backends = [cv.dnn.DNN_BACKEND_OPENCV, cv.dnn.DNN_BACKEND_CUDA]
targets = [cv.dnn.DNN_TARGET_CPU, cv.dnn.DNN_TARGET_CUDA, cv.dnn.DNN_TARGET_CUDA_FP16]

backend_target_pairs = [
    [cv.dnn.DNN_BACKEND_OPENCV, cv.dnn.DNN_TARGET_CPU],
    [cv.dnn.DNN_BACKEND_CUDA,   cv.dnn.DNN_TARGET_CUDA],
    [cv.dnn.DNN_BACKEND_CUDA,   cv.dnn.DNN_TARGET_CUDA_FP16],
    [cv.dnn.DNN_BACKEND_TIMVX,  cv.dnn.DNN_TARGET_NPU],
    [cv.dnn.DNN_BACKEND_CANN,   cv.dnn.DNN_TARGET_NPU]
]

parser = argparse.ArgumentParser(description='Demo for MobileNet V1 & V2.')
parser.add_argument('--input', '-i', type=str, default="../data/dog1_1024_683.jpg",
     help='Usage: Set input path to a certain image, omit if using camera.')
parser.add_argument('--model', '-m', type=str, default='../model/image_classification_mobilenetv1_2022apr.onnx',
     help='Usage: Set model type, defaults to image_classification_mobilenetv1_2022apr.onnx (v1).')
parser.add_argument('--backend', '-b', type=int, default=4,
     help="Chose one pairs of (computation backends, computation devices)")
args = parser.parse_args()

if __name__ == '__main__':

    # Instantiate MobileNet
    model = MobileNet(modelPath=args.model, 
        backendId=backend_target_pairs[args.backend][0], 
        targetId=backend_target_pairs[args.backend][1])
    # Read image and get a 224x224 crop from a 256x256 resized
    image = cv.imread(args.input)
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    image = cv.resize(image, dsize=(256, 256))
    image = image[16:240, 16:240, :]

    # Inference
    result = model.infer(image)

    # Print result
    print('label: {}'.format(result))
