"""
get test videos's meta
"""
import os
import json
import sys

def split_sets(set_path, meta, meta4):
    """
    get frame's meta file
    """
    val_meta = []
    for m, m4 in zip(meta, meta4):
        assert m['H'] == m4['H'] * 4
        assert m['W'] == m4['W'] * 4
        assert m['nframes'] == m4['nframes']
        o = dict(gt_shape = (m['H'], m['W']),
                 x4_shape = (m4['H'], m4['W']),
                 nframes = m['nframes'])
        val_meta.append(o)

    with open(os.path.join(set_path, 'test.json'), 'w') as fid:
        json.dump(dict(videos = val_meta), fid)

def get_videos_meta(frame_width, frame_height, total_frames):
    """
    get frame's meta
    """
    meta = []
    meta4 = []
    
    meta.append(dict(nframes = total_frames, H = frame_height * 4, W = frame_width * 4))
    meta4.append(dict(nframes = total_frames, H = frame_height, W = frame_width))

    return meta, meta4
