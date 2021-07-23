#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

AMCT_CAFFE sample of faster-rcnn model

"""
import os
import argparse
from pathlib import Path

import caffe # pylint: disable=E0401
import amct_caffe as amct # pylint: disable=E0401

import init_paths_for_amct
from faster_rcnn_test import caffe_test
from faster_rcnn_test import do_voc2007_benchmark_test


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Faster-RCNN demo')
    parser.add_argument('--weights_file', dest='weights_file', \
                        help='Specify the weights file of caffe model.', \
                        default=None, type=str)
    parser.add_argument('--model_file', dest='model_file', \
                        help='Specify the model file of caffe model.', \
                        default=None, type=str)
    parser.add_argument('--iterations', dest='iterations', \
                        help='Specify iterations of test', \
                        default=5, type=int)
    parser.add_argument('--pre_test', dest='pre_test', \
                        help='Do test with amct caffe calibration or not', \
                        action='store_true')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]', \
                        default=None, type=int)
    parser.add_argument('--cpu', dest='cpu_mode', \
                        help='Use CPU mode (overrides --gpu)', \
                        action='store_true')
    return parser.parse_args()


def faster_rcnn_args_check(args):
    """Check faster rcnn args
    """
    # --model_file
    if args.model_file is None:
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    faster_rcnn_model_file = os.path.realpath(args.model_file)
    if not Path(faster_rcnn_model_file).exists():
        raise RuntimeError('Must specify a caffe deploy prototxt file')
    args.model_file = faster_rcnn_model_file
    # --weights_file
    if args.weights_file is None:
        raise RuntimeError('Must specify a caffe caffemodel file')
    faster_rcnn_weights_file = os.path.realpath(args.weights_file)
    if not Path(faster_rcnn_weights_file).exists():
        raise RuntimeError('Must specify a caffe caffemodel file')
    args.weights_file = faster_rcnn_weights_file
    # --iterations
    if args.iterations > 5:
        raise RuntimeError('Max iterations on sample dataset is 5')
    if args.cpu_mode and args.gpu_id is not None:
        raise RuntimeError('Cannot run in CPU mode and GPU mode at same time.')


def main():
    """Main function"""
    args = parse_args()
    faster_rcnn_args_check(args)
    if args.cpu_mode:
        args.gpu_id = None
    # User model files
    model_file = args.model_file
    weights_file = args.weights_file
    if args.pre_test:
        caffe_test(model_file, weights_file, args.iterations, gpu_id=args.gpu_id,
                   calibration=False, data_dir='./datasets', is_quantize=False)
        print('[AMCT][INFO]Run faster_rcnn without quantize success!')
        m_ap = do_voc2007_benchmark_test(model_file, weights_file, args.gpu_id)
        print('[AMCT][INFO]Run faster_rcnn without quantize success, and mAP is {}'.format(m_ap))
        return

    # amct.set_cpu_mode() or set_gpu_mode() decides whether using CPU/GPU
    # to do weights calibration, but activation calibration is controled
    # by caffe APIs: caffe.set_mode_cpu() or set_mode_gpu().
    # Need to set amct mode before the whole calibration process, default
    # using CPU mode to do weights calibration.
    # amct.set_gpu_mode() does not set which GPU card to use. Users can set
    # GPU card in two ways:
    # 1) use pycaffe API set_device(gpu_id)
    # 2) use environment variable CUDA_VISIBLE_DEVICES
    if args.gpu_id is not None:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        amct.set_gpu_mode()

    # Quantize configurations
    config_json_file = './config.json'
    skip_layers = []
    batch_num = 1
    amct.create_quant_config(config_json_file, model_file, weights_file, \
        skip_layers, batch_num)

    # Phase0: Init amct task
    scale_offset_record_file = './tmp/scale_offset_record/record.txt'
    graph = amct.init(config_json_file, model_file, weights_file, scale_offset_record_file)

    # Phase1: Do conv+bn+scale fusion, weights calibration and fake quant,
    #         insert quant and dequant layer.
    modified_model_file = os.path.realpath('./tmp/modified_model.prototxt')
    modified_weights_file = os.path.realpath('./tmp/modified_model.caffemodel')
    amct.quantize_model(graph, modified_model_file, modified_weights_file)

    # Phase2: run caffe model to do activation calibration
    caffe_test(modified_model_file, modified_weights_file, batch_num, \
               gpu_id=args.gpu_id, calibration=True, data_dir='./datasets',
               is_quantize=False)

    # Phase3: Save final model, one for caffe do fake quant test,
    #         one deploy model for GE
    amct.save_model(graph, 'Both', './results/faster_rcnn')

    # Phase4: do final fake quant model test
    fake_quant_model = './results/faster_rcnn_fake_quant_model.prototxt'
    fake_quant_weights = './results/faster_rcnn_fake_quant_weights.caffemodel'
    caffe_test(fake_quant_model, fake_quant_weights, \
               args.iterations, gpu_id=args.gpu_id, calibration=False, data_dir='./datasets',
               is_quantize=True)
    print('[AMCT][INFO]Run faster_rcnn with quantize success!')
    m_ap = do_voc2007_benchmark_test(fake_quant_model, fake_quant_weights, args.gpu_id)
    print('[AMCT][INFO]Run faster_rcnn with quantize success, and mAP is {}!'.format(m_ap))

if __name__ == '__main__':
    main()
