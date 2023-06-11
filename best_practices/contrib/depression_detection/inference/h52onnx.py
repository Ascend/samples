# -*- coding: utf-8 -*-
import os
import onnxmltools
import onnx
from keras.models import load_model
from MagicONNX.magiconnx import OnnxGraph
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--h5_path", default="")
parser.add_argument("--onnx_path", default="")
args = parser.parse_args()

def h5_to_onnx(input_h5, output_onnx):
    model = load_model(input_h5)
    onnx_model = onnxmltools.convert_keras(model, model.name)
    onnx.save_model(onnx_model, output_onnx)

h5_to_onnx(args.h5_path, 'keras_model.onnx')

graph = OnnxGraph('keras_model.onnx')

graph.keep_default_domain()

graph.save(args.onnx_path)

os.remove('keras_model.onnx')

