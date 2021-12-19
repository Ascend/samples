"""
build test dataloader
"""
import os
import glob
import imageio
import numpy as np
import json

class TestMinibatch(object):
    """
    test dataloader struct
    """
    __instance = None
    def __new__(cls, *args, **kwargs):
        if cls.__instance is None:
            TestMinibatch.__instance = super().__new__(cls)
        return cls.__instance

    def __init__(self, data_dir, set_file='val.json', batch_size=1, num_frames=5, scale=4):
        self.batch_size = batch_size
        self.num_frames = num_frames
        self.scale = scale

        set_file = os.path.join(data_dir, set_file)
        with open(set_file, 'r') as fid:
            meta = json.load(fid)

        self.lrcliplist = []
        for vid in meta['videos']:
            in_path = os.path.join(data_dir, 'images')
            inList = sorted(glob.glob(os.path.join(in_path, '*.png')))

            max_frame = len(inList)
            for i in range(max_frame):
                index = np.array([k for k in range(i - self.num_frames // 2, i + self.num_frames // 2 + 1)])
                index = np.clip(index, 0, max_frame - 1).tolist()
                self.lrcliplist.append([inList[k] for k in index])

        self.total_samples = len(self.lrcliplist)
        self.index = list(range(self.total_samples))
        self.cur = 0

    def __iter__(self):
        return self

    def __next__(self):
        return self.get_next()

    def __len__(self):
        return self.total_samples

    def get_next(self):
        """
        get next batch
        """
        if self.cur + self.batch_size > self.total_samples:
            return None

        lrlist = [self.lrcliplist[idx] for idx in self.index[self.cur:self.cur + self.batch_size]]
        lr_names = [self.lrcliplist[idx][self.num_frames // 2] 
                          for idx in self.index[self.cur:self.cur + self.batch_size]]
        for pervid in lrlist:
            lr = [np.array([imageio.imread(perimg)[..., :3] / 255. for perimg in pervid]).astype(np.float32)]
        lr = np.array(lr)

        self.cur += self.batch_size

        return lr_names, lr

def build_test_dataloader(batch_size, scale, set_file, num_frames, data_config):
    """
    build test dataloader
    """
    dataloader = TestMinibatch(
        data_dir=data_config,
        set_file=set_file,
        batch_size=batch_size,
        num_frames=num_frames,
        scale=scale
    )
    return dataloader