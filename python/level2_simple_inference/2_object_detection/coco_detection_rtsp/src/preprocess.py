"""preprocess"""
import os
import time
import configparser
from multiprocessing import Process, Queue, Manager
import queue
import numpy as np
import sys

import acl
from acllite_imageproc import AclLiteImageProc
import videocapture as video
import acllite_utils as utils
import constants as const
from acllite_logger import log_error, log_info

STATUS_PREPROC_INIT = 1
STATUS_PREPROC_RUNNING = 2
STATUS_PREPROC_EXIT = 3
STATUS_PREPROC_ERROR = 4

class PreProcData(object):
    """preprocdata"""
    def __init__(self, rtsp_channel, width, height, 
                 resized_image, jpg_image, display):
        self.channel = rtsp_channel
        self.frame_width = width
        self.frame_height = height
        self.resized_image = resized_image
        self.jpg_image = jpg_image
        self.display = display

class Preprocess(object): 
    """preprocess"""
    def __init__(self, stream_name, channel, resize_width, resize_height):
        self._stream_name = str(stream_name)
        self._channel = int(channel)         
        self._resize_width = resize_width
        self._resize_height = resize_height
        self._status = STATUS_PREPROC_INIT
        self._display = False
        self._dvpp = None
        self._cap = None
        self._context = None
        self._image_queue =  queue.Queue(64)

    def _start(self):
        thread_id, ret = acl.util.start_thread(self._thread_entry, [])            
        utils.check_ret("acl.util.start_thread", ret)

        log_info("Start sub thread ok, wait init...")
        while self._status == STATUS_PREPROC_INIT:
            time.sleep(0.001)
        log_info("Status changed to ", self._status)
        
        while self._status == STATUS_PREPROC_RUNNING:
            if self._image_queue.qsize() > 0:
                break
            time.sleep(0.001)
        
        return self._status != STATUS_PREPROC_ERROR

    def _thread_entry(self, args_list):     
        self._context, ret = acl.rt.create_context(0)
        utils.check_ret("acl.rt.create_context", ret)
        self._cap = video.VideoCapture(self._stream_name)
        self._dvpp = AclLiteImageProc() 
        self._status = STATUS_PREPROC_RUNNING
        frame_cnt = 0
        while self._status == STATUS_PREPROC_RUNNING: 
            ret, image = self._cap.read()
            if ret:
                log_error("Video %s decode failed" % (self._stream_name))
                self._status = STATUS_PREPROC_ERROR
                break

            if (image is None) and self._cap.is_finished():
                log_info("Video %s decode finish" % (self._stream_name))
                self._status = STATUS_PREPROC_EXIT
                break

            if image and (int(frame_cnt) % 5 == 0):
                self._process_frame(image)  
            frame_cnt += 1

        self._thread_exit()        

    def _process_frame(self, frame):
        resized_image = self._dvpp.crop_and_paste(frame, frame.width, frame.height, 
                                                  self._resize_width, self._resize_height)
        if resized_image is None:
            log_error("dvpp resize image failed")
            return

        jpg_image = None
        if self._display:
            jpg_image = self._dvpp.jpege(frame)
            if jpg_image is None:
                log_error("dvpp jpege failed")
                return

        data = PreProcData(self._channel, frame.width, frame.height, 
                           resized_image, jpg_image, self._display) 
              
        self._image_queue.put(data)

    def _thread_exit(self):
        self._status = STATUS_PREPROC_EXIT
        log_info("Channel %d thread exit..." % (self._channel))
        if self._dvpp is not None:
            del self._dvpp
            self._dvpp = None

        if self._cap is not None:
            while self._cap._dextory_dvpp_flag == False:
                time.sleep(0.001)
            del self._cap
            self._cap = None

        if self._context is not None:
            acl.rt.destroy_context(self._context)
            self._context = None
        log_info("Channel %d thread exit ok" % (self._channel))

    def set_display(self, display):
        """set display"""
        self._display = display

    def is_finished(self):
        """
        Function description:
            Judge whether the process is completed
        Parameter:
            none
        Return Value:
            True or False
        Exception Description:
            none
        """
        return self._status > STATUS_PREPROC_RUNNING
    
    def get_data(self):
        """
        The method for getting data 
        """
        if self._status >= STATUS_PREPROC_EXIT:            
            return False, None
        elif self._status == STATUS_PREPROC_INIT:
            ret = self._start()
            if ret == False:
                log_error("decode channel %d failed" % (self._channel))
                return False, None

        if self._image_queue.empty():
            return True, None

        preproc_data = self._image_queue.get_nowait()
        return True, preproc_data  

    def __del__(self):
        self._thread_exit()


