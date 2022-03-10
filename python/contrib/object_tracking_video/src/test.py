
"""
Copyright 2021 Huawei Technologies Co., Ltd.
Copyright (c) 2020 YifuZhang

Licensed under the Apache License, Version 2.0 (the "License");
You may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
# TEST one sinle image
# This script uses one sinle image as input, generate bounding box with id
# To test on video, please use main.py
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
import cv2
import shutil
import argparse
import numpy as np

sys.path.append("../../../common/")

from utils.dataloader import loader # LoadVideo, LoadImages
from multitracker import JDETracker
from utils.timer import Timer
from utils import visualization as vis

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource 

import math
import operator
import functools 
from PIL import Image

# Copyright (R) @huawei.com, all rights reserved
# -*- coding:utf-8 -*-
# CREATED:  2020-12-11 10:12:13
def image_contrast(image1, image2):
    """
    Verify that the pictures are the same
    """
    file1 = Image.open(image1)
    file2 = Image.open(image2)
    h1 = file1.histogram()
    h2 = file2.histogram()
    ret = math.sqrt(functools.reduce(operator.add, list(map(lambda a, b: (a - b) ** 2, h1, h2))) / len(h1))
    return ret


def test(args):
    # Step 1: initialize ACL and ACL runtime 
    acl_resource = AclLiteResource()

    # 1.2: one line of code, call the 'init' function of the AclLiteResource object, to initilize ACL and ACL runtime 
    acl_resource.init()

    # Step 2: Load models 
    mot_model = AclLiteModel('../model/mot_v2.om')

    dataloader = loader.LoadImages(args.test_img)

    # initialize tracker
    tracker = JDETracker(args, mot_model, frame_rate=30)
    timer = Timer()
    results = []
    
    # img:  h w c; 608 1088 3
    # img0: c h w; 3 608 1088
    for frame_id, (path, img, img0) in enumerate(dataloader):
        if frame_id % 20 == 0 and frame_id != 0:
            print('Processing frame {} ({:.2f} fps)'.format(frame_id, 1. / max(1e-5, timer.average_time)))

        # run tracking, start tracking timer 
        timer.tic()

        # list of Tracklet; see multitracker.STrack
        online_targets = tracker.update(np.array([img]), img0)

        # prepare for drawing, get all bbox and id
        online_tlwhs = []
        online_ids = []
        for t in online_targets:
            tlwh = t.tlwh
            tid = t.track_id
            vertical = tlwh[2] / tlwh[3] > 1.6
            if tlwh[2] * tlwh[3] > args.min_box_area and not vertical:
                online_tlwhs.append(tlwh)
                online_ids.append(tid)
        timer.toc()

        # draw bbox and id
        online_im = vis.plot_tracking(img0, online_tlwhs, online_ids, frame_id=frame_id,
                                        fps=1. / timer.average_time)
        cv2.imwrite(os.path.join('../data', 'test_output.jpg'), online_im)

    # verify if result is expected
    result = image_contrast('../data/test_output.jpg', args.verify_img)
    print(result)
    if (result > 420 or result < 0):
        print("Similarity Test Fail!")
        sys.exit(1)
    else:
        print("Similarity Test Pass!")
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--conf_thres', type=float, default=0.35, help='confidence thresh for tracking')
    parser.add_argument('--track_buffer', type=int, default=30, help='tracking buffer')
    parser.add_argument('--min_box_area', type=float, default=100, help='filter out tiny boxes')
    parser.add_argument('--K', type=int, default=100, help='Max number of detection per image')

    parser.add_argument('--test_img', type=str, default='../data/test.jpg', help='path to the test image')
    parser.add_argument('--verify_img', type=str, default='../data/verify.jpg', help='path to the expected output image')

    args = parser.parse_args()
    args.mean = [0.408, 0.447, 0.470]
    args.std = [0.289, 0.274, 0.278]
    args.down_ratio = 4
    args.num_classes = 1

    test(args) 