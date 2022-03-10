"""postprocess"""
import os
import time
import configparser
import queue
import numpy as np
import sys
import acl
import acllite_utils as utils
import constants as const
from acllite_logger import log_error, log_info

class DetectData(object):
    """detecdata"""
    def __init__(self, frame_data, detect_results):
        self.frame_data = frame_data
        self.detect_results = detect_results

class Postprocess(object): 
    """post"""
    def __init__(self, detect_model):
        self._detector = detect_model
        self._channel = None         
        self._data_queue =  queue.Queue(64)
        self._context = None
        self._start()
        self._exit = False

    def _start(self):
        thread_id, ret = acl.util.start_thread(self._thread_entry, [])            
        utils.check_ret("acl.util.start_thread", ret)

    def _thread_entry(self, args_list):   
        self._context, ret = acl.rt.create_context(0)
        utils.check_ret("acl.rt.create_context", ret)
        
        ret = True
        while ret: 
            data = self._data_queue.get()
            if isinstance(data, DetectData):
                ret = self._process_detect_data(data.detect_results, 
                                                data.frame_data)
            elif isinstance(data, str):
                log_info("Post process thread exit")
                self._exit = True
                ret = False
            else: 
                log_error("post process thread receive unkonow data")    
            
    def _process_detect_data(self, detect_results, frame_data):
        results_list = self._detector.post_process(detect_results, frame_data)  
        ret = True                                                 
        return ret        

    def process(self, data, detect_results):
        """process"""
        detect_data = DetectData(data, detect_results)
        self._data_queue.put(detect_data)

    def exit(self):
        """exit"""
        self._data_queue.put("exit")
        while self._exit == False:
            time.sleep(0.001)
       
