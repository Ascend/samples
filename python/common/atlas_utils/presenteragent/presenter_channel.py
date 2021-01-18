# !/usr/bin/env python
# -*- coding:utf-8 -*-
import time
import configparser
from multiprocessing import Process, Queue, Manager
import queue
import numpy as np
import sys
#sys.path.append("..")

import acl
from atlas_utils.constants import *
from atlas_utils.acl_logger import log_error, log_info
from atlas_utils.data_buf import DataBuf
from atlas_utils.acl_image import AclImage


from .presenter_datatype import *
from . import presenter_agent as agent
from . import presenter_message as pm


class PresenterChannel():
    def __init__(self, server_ip, port, name='video', type=CONTENT_TYPE_VIDEO):
        self._server_ip = server_ip
        self._port = port
        self._type = type
        self._name = name
        self.agent_msg_queue = Queue()
        self.open_status = Manager().Value('i', STATUS_DISCONNECT)
        self.data_respone_counter =  Manager().Value('i', 0)
        self._send_counter = 0
        self._send_buffer = queue.Queue(64)
        self.send_cnt = 0
        self.relase_cnt = 0

    def startup(self):
        agent_process = Process(target=agent.StartPresenterAgent,
                                args=(self.agent_msg_queue, self._server_ip,self._port, 
                                      self.open_status, self.data_respone_counter))
        agent_process.start()
        time.sleep(0.5)
        self._send_open_channel_request(self._name, self._type)        
        return self._wait_open_status(STATUS_OPENED)        

    def _wait_open_status(self, listen_status):
        ret = STATUS_ERROR
        for i in range(0, 100):
            time.sleep(0.001)
            if self.open_status.value == listen_status:
                log_info("Open status is %d now"%(listen_status))
                ret = STATUS_OK
                break
        return ret
        

    def send_message(self, data):
        self.agent_msg_queue.put(data)
        self._send_counter += 1

    def _send_open_channel_request(self, channel_name, content_type):
        request_msg = pm.open_channel_request(channel_name, content_type)
        self.send_message(request_msg)

    def _release_send_success_data(self):
        release_num = self._send_buffer.qsize() - \
                      (self._send_counter - self.data_respone_counter.value)
        if release_num > 0:
            for i in range(0, release_num):                
                data = self._send_buffer.get_nowait()
                if isinstance(data, DataBuf):
                    data.destroy()
                data = None
                self.relase_cnt += 1

    def send_detection_data(self, image_width, image_height,
                            image, detection_result):
        if self._send_buffer.full() is True:
            log_error("Send detection data failed for buffer is full")
            return False

        image_data = None
        if isinstance(image, AclImage):
            image_data = DataBuf(image.data(), image.size).copy_to_local()
        elif isinstance(image, np.ndarray):
            image_data = image            
        else:
            log_error("Invalid data to send, ", image)    
            return False 

        request_msg = pm.image_frame_request(image_width, image_height,
                                             image_data.tobytes(),
                                             detection_result)      
        self.send_message(request_msg)   
        self._send_buffer.put(image_data)     
        self._release_send_success_data()

        return True

    def send_image(self, image_width, image_height, image):
        detection_result = []
        return self.send_detection_data(image_width, image_height, 
                                        image, detection_result)

    def _send_heart_beat_message(self):
        msg = pm.heartbeat_message()
        self.send_message(msg)

    def close(self):
        if self.open_status == STATUS_EXITTED:    
            return

        log_info("Presenter channel close...")
        eos = FinishMsg("exit")
        self.send_message(eos)
        while self.agent_msg_queue.qsize() > 0:
            self._release_send_success_data()
            time.sleep(0.001)
        self.open_status = STATUS_EXITTED       

    def __del__(self):
        self.close()


def get_channel_config(config_file):
    config = configparser.ConfigParser()
    config.read(config_file)
    presenter_server_ip = config['baseconf']['presenter_server_ip']
    port = int(config['baseconf']['presenter_server_port'])
    channel_name = config['baseconf']['channel_name']
    content_type = int(config['baseconf']['content_type'])
	
    log_info("presenter server ip %s, port %d, channel name %s, "
             "type %d"%(presenter_server_ip, port, channel_name, content_type))
    return presenter_server_ip, port, channel_name, content_type

def open_channel(config_file):
    server_ip, port, channel_name, content_type = get_channel_config(config_file)
    channel = PresenterChannel(server_ip, port, channel_name, content_type)
    ret = channel.startup()
    if ret:
        log_error("ERROR:Open channel failed")
        return None
    return channel
