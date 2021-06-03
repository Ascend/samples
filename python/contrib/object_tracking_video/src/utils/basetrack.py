"""
BaseTrack Structure Definition 
"""
import numpy as np
from collections import OrderedDict


class TrackState(object):
    """
    State Definition
    """
    New = 0
    Tracked = 1
    Lost = 2
    Removed = 3


class BaseTrack(object):
    """
    Define Basic Track Structure
    """
    
    _count = 0

    track_id = 0
    is_activated = False
    state = TrackState.New

    history = OrderedDict()
    features = []
    curr_feature = None
    score = 0
    start_frame = 0
    frame_id = 0
    time_since_update = 0

    # multi-camera
    location = (np.inf, np.inf)

    @property
    def end_frame(self):
        """
        return frame id
        """
        return self.frame_id

    @staticmethod
    def next_id():
        """
        return number count
        """
        BaseTrack._count += 1
        return BaseTrack._count

    def activate(self, *args):
        """activate"""
        raise NotImplementedError

    def predict(self):
        """predict"""
        raise NotImplementedError

    def update(self, *args, **kwargs):
        """update"""
        raise NotImplementedError

    def mark_lost(self):
        """mark_lost"""
        self.state = TrackState.Lost

    def mark_removed(self):
        """mark_removed"""
        self.state = TrackState.Removed