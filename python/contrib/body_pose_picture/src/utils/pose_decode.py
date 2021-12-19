"""
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import cv2
import numpy as np
import math
import os
import sys

heatmap_width = 92
heatmap_height = 92
# Joints Explained
# 14 joints:
# 0-right shoulder, 1-right elbow, 2-right wrist, 3-left shoulder, 4-left elbow, 5-left wrist, 
# 6-right hip, 7-right knee, 8-right ankle, 9-left hip, 10-left knee, 11-left ankle, 
# 12-top of the head and 13-neck

#                      12                     
#                      |
#                      |
#                0-----13-----3
#               /     / \      \
#              1     /   \      4
#             /     /     \      \
#            2     6       9      5
#                  |       |
#                  7       10
#                  |       |
#                  8       11

JOINT_LIMB = [[0, 1], [1, 2], [3, 4], [4, 5], 
              [6, 7], [7, 8], [9, 10], [10, 11], 
              [12, 13], [13, 0], [13, 3], [13, 6], [13, 9]]
COLOR = [[0, 255, 255], [0, 255, 255], [0, 255, 255], [0, 255, 255],
         [0, 255, 0], [0, 255, 0], [0, 255, 0], [0, 255, 0],
         [0, 0, 255], [255, 0, 0], [255, 0, 0], [255, 0, 0], [255, 0, 0]]

def decode_pose(heatmaps, scale, image_original):
    """obtain joint list from heatmap"""
    
    # joint_list: a python list of joints, joint_list[i] is an numpy array with the (x,y) coordinates of 
    # the i'th joint (refer to the 'Joints Explained' in this file, e.g., 0th joint is right shoulder)  
    joint_list = [peak_index_to_coords(heatmap) * scale for heatmap in heatmaps]
    
    # plot the pose on original image
    canvas = image_original
    for idx, limb in enumerate(JOINT_LIMB):
        joint_from, joint_to = joint_list[limb[0]], joint_list[limb[1]]
        canvas = cv2.line(canvas, tuple(joint_from.astype(int)), 
                          tuple(joint_to.astype(int)), color=COLOR[idx], thickness=4)
    return canvas  

def peak_index_to_coords(peak_index):
    """
    @peak_index is the index of max value in flatten heatmap
    This function convert it back to the coordinates of the original heatmap 
    """
    peak_coords = np.unravel_index(int(peak_index), (heatmap_height, heatmap_width))
    return np.flip(peak_coords)
