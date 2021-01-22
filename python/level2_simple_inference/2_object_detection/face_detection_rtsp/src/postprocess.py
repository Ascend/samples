import os
import time
import configparser
import queue
import numpy as np
import sys

import acl
import atlas_utils.utils as utils
import atlas_utils.constants as const
from atlas_utils.acl_logger import log_error, log_info
from atlas_utils.presenteragent import presenter_channel

class DetectData():
    def __init__(self, frame_data, detect_results):
        self.frame_data = frame_data
        self.detect_results = detect_results


class Postprocess(): 
    def __init__(self, detect_model):
        self._detector = detect_model
        self._channel = None         
        self._data_queue =  queue.Queue(64)
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
        if frame_data.display and frame_data.jpg_image:
            ret = self._channel.send_detection_data(frame_data.frame_width, 
                                                    frame_data.frame_height, 
                                                    frame_data.jpg_image, 
                                                    results_list)
            if ret == False:
                log_error("Send data to presenter server failed")
        return ret        

    def create_presenter_channel(self, config_file):        
        self._display = True
        self._channel = presenter_channel.open_channel(config_file)
        if self._channel == None:
            log_error("Open presenter channel failed")
            return False
        
        return True

    def process(self, data, detect_results):
        detect_data = DetectData(data, detect_results)
        self._data_queue.put(detect_data)

    def exit(self):
        self._data_queue.put("exit")
        while self._exit == False:
            time.sleep(0.001)
       



