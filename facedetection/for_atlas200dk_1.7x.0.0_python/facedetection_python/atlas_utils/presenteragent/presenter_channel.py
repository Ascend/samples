# !/usr/bin/env python
# -*- coding:utf-8 -*-
import ctypes
from ctypes import *
import time
import configparser
from multiprocessing import Process, Queue, Manager
import queue
import numpy as np
import sys
sys.path.append("..")

import acl
from constants import *
from lib.atlasutil_so import libatlas

from .presenter_datatype import *
from . import presenter_agent as agent
from . import presenter_message as pm


class DataBufC(Structure):
    _fields_ = [
        ('size',        c_int),
        ('data',        POINTER(c_ubyte))
    ]

class DataBuf():
    def __init__(self, data, data_size):
        self.data = data
        self.size = data_size
        self.nparray = None

    def copy_to_local(self):
        src_data = DataBufC()
        src_data.data = cast(self.data, POINTER(c_ubyte))
        src_data.size = self.size
        dest_data = DataBufC()
        ret = libatlas.CopyDataToLocal(byref(dest_data), byref(src_data))
        if ret:
            print("Copy data to local failed")
            return None

        return DataBuf(dest_data.data, dest_data.size)
    
    def tobytes(self):
        self.nparray = np.frombuffer((ctypes.c_ubyte * self.size).from_address(ctypes.addressof(self.data.contents)), dtype=np.uint8)
        return self.nparray.tobytes()

    def destroy(self):
        data_buf = DataBufC()
        data_buf.data = cast(self.data, POINTER(c_ubyte))
        data_buf.size = self.size
        libatlas.ReleaseDataBuf(byref(data_buf))
        self.data = None
        self.size = 0
            

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
            time.sleep(0.1)
            if self.open_status.value == listen_status:
                print("Open status is %d now"%(listen_status))
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
                data.destroy()
                data = None
                self.relase_cnt += 1
        #print("Released send success images ", self.relase_cnt)

    def send_detection_data(self, image_width, image_height,
                            image_data, detection_result):
        if self._send_buffer.full() is True:
            print("ERROR:Send detection data failed for buffer is full")
            return False

        image_buf = DataBuf(image_data.data(), image_data.size).copy_to_local()
        request_msg = pm.image_frame_request(image_width, image_height,
                                             image_buf.tobytes(), detection_result)
        self.send_message(request_msg)
        self._send_buffer.put(image_buf)
        self._release_send_success_data()

        return True

    def _send_heart_beat_message(self):
        msg = pm.heartbeat_message()
        self.send_message(msg)

    def __del__(self):
        self.open_status.value = STATUS_EXITING
        print("Presenter channel close...")
        self._send_heart_beat_message()
        if STATUS_OK == self._wait_open_status(STATUS_EXITTED):
            print("Presenter channel closed")
        else:
            print("Presenter channel close failed for presenter agent no response")


def get_presenter_server_addr(config_file):
    config = configparser.ConfigParser()
    config.read(config_file)
    presenter_server_ip = config['baseconf']['presenter_server_ip']
    port = int(config['baseconf']['presenter_server_port'])
	
    print("presenter server ip %s, port %d"%(presenter_server_ip, port))
    return presenter_server_ip, port

def open_channel(config_file, channel_name='video', channel_type = CONTENT_TYPE_VIDEO):
    server_ip, port = get_presenter_server_addr(config_file)
    channel = PresenterChannel(server_ip, port, channel_name, channel_type)
    ret = channel.startup()
    if ret:
        print("ERROR:Open channel failed")
        return None
    return channel
