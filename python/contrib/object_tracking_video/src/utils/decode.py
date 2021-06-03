"""
Post-processing
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import numpy as np
from numpy.lib.stride_tricks import as_strided

def mot_decode(heat, wh, reg, id_feature, conf_thres):
    """
    Post-process the output from network & generate the detection
    Returns:
        dets: n * 6 matrix where n is number of detections
                bbox_top_left x, y; bbox_bottom_right x, y; conf_score; class (all zeros [only human])
        inds: indices of detection in flatten array (152*272)
    """

    mask = heat > conf_thres
    ys, xs = np.nonzero(mask)
    xs = xs.astype(float)
    ys = ys.astype(float)
    # print(xs, ys)
    
    scores = heat[mask]
    wh = wh[mask]
    reg = reg[mask]
    id_feature = id_feature[mask]
    xs += reg[:, 0]
    ys += reg[:, 1]

    clses = np.zeros(scores.shape[0])

    detections = np.stack([xs - wh[:, 0],
                           ys - wh[:, 1],
                           xs + wh[:, 2],
                           ys + wh[:, 3],
                           scores,
                           clses], axis=0)

    return detections.T, id_feature