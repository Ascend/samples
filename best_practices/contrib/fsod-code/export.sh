#!/bin/bash


python3 tools/deploy/export_model.py --config-file configs/Custom-detection/faster_rcnn_R_101_FPN_base1_jinxiang.yaml --output ./output --export-method tracing --format onnx MODEL.WEIGHTS model_final_1010.pth MODEL.DEVICE cpu

mv output/model.onnx model_py1.8.onnx