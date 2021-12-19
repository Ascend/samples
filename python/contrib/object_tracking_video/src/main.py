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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
import cv2
import argparse
import numpy as np

sys.path.append("../../../common/")

from utils.dataloader import LoadVideo, LoadImages
from multitracker import JDETracker
from utils.timer import Timer
from utils import visualization as vis

from acllite_model import AclLiteModel
from acllite_resource import AclLiteResource 

def mkdir_if_missing(d):
    """
    create directory if not exist
    """
    if not os.path.exists(d):
        os.makedirs(d)


def main(args):
    """main"""
    # Step 1: initialize ACL and ACL runtime 
    acl_resource = AclLiteResource()

    # 1.2: one line of code, call the 'init' function of the AclLiteResource object, to initilize ACL and ACL runtime 
    acl_resource.init()

    # Step 2: Load models 
    mot_model = AclLiteModel('../model/mot_v2.om')

    # Create output dir if not exist; default outputs
    result_root = args.output_root if args.output_root != '' else '.'
    mkdir_if_missing(result_root)

    video_name = os.path.basename(args.input_video).replace(' ', '_').split('.')[0]

    # setup dataloader, use LoadVideo or LoadImages
    dataloader = LoadVideo(args.input_video, (1088, 608))
    # result_filename = os.path.join(result_root, 'results.txt')
    frame_rate = dataloader.frame_rate

    # dir for output images; default: outputs/'VideoFileName'
    save_dir = os.path.join(result_root, video_name)    

    mkdir_if_missing(save_dir)

    # initialize tracker
    tracker = JDETracker(args, mot_model, frame_rate=frame_rate)
    timer = Timer()
    results = []
    
    print("Results will be saved at {}".format(save_dir))
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
        cv2.imwrite(os.path.join(save_dir, '{:05d}.jpg'.format(frame_id)), online_im)


    if args.output_type == 'video':
        output_video_path = os.path.join(result_root, os.path.basename(args.input_video).replace(' ', '_'))
        cmd_str = 'ffmpeg -f image2 -i {}/%05d.jpg -b 5000k -c:v mpeg4 {}'.format(save_dir, output_video_path)
        os.system(cmd_str)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--conf_thres', type=float, default=0.35, help='confidence thresh for tracking')
    parser.add_argument('--track_buffer', type=int, default=30, help='tracking buffer')
    parser.add_argument('--min_box_area', type=float, default=100, help='filter out tiny boxes')
    parser.add_argument('--K', type=int, default=100, help='Max number of detection per image')

    parser.add_argument('--input_video', type=str, default='inputs/london_t.mp4', help='path to the input video')
    parser.add_argument('--output_root', type=str, default='../outputs', help='expected output root path')
    parser.add_argument('--output_type', type=str, default='images', help='images or video (require `ffmpeg`)')
    
    args = parser.parse_args()
    args.mean = [0.408, 0.447, 0.470]
    args.std = [0.289, 0.274, 0.278]
    args.down_ratio = 4
    args.num_classes = 1

    main(args) 