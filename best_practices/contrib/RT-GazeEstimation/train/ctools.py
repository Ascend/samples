import numpy as np
import sys
import time
import os
import json
from easydict import EasyDict as edict

class TimeCounter:
    # Create an time counter.
    # To count the rest time.

    # Input the total times.
    def __init__(self, total):
      self.total = total
      self.cur = 0
      self.begin = time.time()

    def step(self):
      end = time.time() 
      self.cur += 1
      used = (end - self.begin)/self.cur
      rest = self.total - self.cur

      return np.max(rest * used, 0)
         

def readfolder(data, specific=None, reverse=False):

    """" 
    Traverse the folder 'data.label' and read data from all files in the folder.
    
    Specific is a list, specify the num of extracted file.

    When reverse is True, read the files which num is not in specific. 
    """

 
    folders = os.listdir(data.label)
    folders.sort()

    folder = folders
    if specific is not None:
        if reverse:
            num = np.arange(len(folders))
            specific = list(filter(lambda x: x not in specific, num))
        
        folder = [folders[i] for i in specific]

    data.label = [os.path.join(data.label, j) for j in folder]

    return data, folders


def DictDumps(content):
    return json.dumps(content,  ensure_ascii=False, indent=4)


def GetLR(optimizer):
    LR = optimizer.state_dict()['param_groups'][0]['lr']
    return LR

