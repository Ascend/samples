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

AMCT_CAFFE sample of faster-rcnn model test

"""
import os
import matplotlib.pyplot as plt # pylint: disable=E0401
import cv2 # pylint: disable=E0401
from fast_rcnn.config import cfg # pylint: disable=E0401
from fast_rcnn.test import im_detect # pylint: disable=E0401
import caffe # pylint: disable=E0401
from tools.demo import demo # pylint: disable=E0401


def do_calibration(modified_model_file, modified_weights_file, batch_num):
    """do calibration"""
    if batch_num > 5:
        raise RuntimeError("Max batch_num is 5, {} overflow".format(batch_num))
    net = caffe.Net(modified_model_file, modified_weights_file, caffe.TEST)
    im_names = ['000456.jpg', '000542.jpg', '001150.jpg',
                '001763.jpg', '004545.jpg']
    for index in range(batch_num):
        im_detect(net, cv2.imread(os.path.join(cfg.DATA_DIR, im_names[index])))


def do_test(fake_quant_model_file, fake_quant_weights_file, batch_num, is_quantize):
    """do fake quant model test"""
    if batch_num > 5:
        raise RuntimeError("Max batch_num is 5, {} overflow".format(batch_num))
    net = caffe.Net(fake_quant_model_file, fake_quant_weights_file, caffe.TEST)
    im_names = ['000456.jpg', '000542.jpg', '001150.jpg',
                '001763.jpg', '004545.jpg']
    for index in range(batch_num):
        demo(net, im_names[index], is_quantize)


def caffe_test(model_file, weights_file, batch_num, # pylint: disable=R0913
               gpu_id=None, calibration=False,
               data_dir=None, is_quantize=False):
    """do caffe model eval"""
    if gpu_id is None:
        cfg.USE_GPU_NMS = False
        caffe.set_mode_cpu()
    else:
        caffe.set_mode_gpu()
        caffe.set_device(gpu_id)
        cfg.GPU_ID = gpu_id

    if data_dir is None:
        raise RuntimeError("Must specify a data_dir")
    cfg.DATA_DIR = os.path.realpath(data_dir)

    cfg.TEST.HAS_RPN = True

    if model_file is None:
        raise RuntimeError('Must specify a caffe model file')
    model_file = os.path.realpath(model_file)
    if weights_file is None:
        raise RuntimeError('Must specify a caffe weights file')
    weights_file = os.path.realpath(weights_file)

    if calibration:
        do_calibration(model_file, weights_file, int(batch_num))
    else:
        do_test(model_file, weights_file, int(batch_num), is_quantize)
        print('[AMCT][INFO]Show detect result in pictures, ' \
            'close all matplotlib show windows to end sample.')
        if is_quantize:
            print('[AMCT][INFO]See detect results in "quant_detect_results".')
        else:
            print('[AMCT][INFO]See detect results in "pre_detect_results".')
        plt.show()


class Args(): # pylint: disable=R0902, R0903
    """docstring for Args"""
    def __init__(self):
        self.gpu_id = 0
        self.prototxt = None
        self.caffemodel = None
        self.cfg_file = 'experiments/cfgs/faster_rcnn_alt_opt.yml'
        self.wait = True
        self.imdb_name = 'voc_2007_test'
        self.comp_mode = False
        self.set_cfgs = None
        self.vis = False
        self.max_per_image = 100


def do_voc2007_benchmark_test(model_file, weights_file, gpu_id):
    """do VOC2007 benchmark test"""
    try:
        from tools.test_net import test_voc_2007 # pylint: disable=C0415
    except ImportError as exception:
        raise RuntimeError('Not support do benchamrk test, please rerun ' \
            'init_env.sh with "with_benchmark":{}'.format(exception)) from exception
    args = Args()
    args.gpu_id = gpu_id
    args.prototxt = model_file
    args.caffemodel = weights_file
    cfg.DATA_DIR = './datasets'
    m_ap = test_voc_2007(args)
    return m_ap
