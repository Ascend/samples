# !/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import time
import configparser
from multiprocessing import Process, Queue, Manager
import queue
import numpy as np
import sys
#sys.path.append("..")

import acl
from atlas_utils.constants import *
from atlas_utils.utils import *


DEVICE_ID = 0
IMAGE_INPUT_WIDTH = 1280
IMAGE_INPUT_HEIGHT = 720

VENC_STATUS_ERROR = -1
VENC_STATUS_INIT = 1
VENC_STATUS_WORK = 1
VENC_STATUS_EXIT = 2

SUBSCRIBE_TIMEOUT = 1000

SUCCESS = 0
FAILED = 1
ACL_ERROR_NONE = 0

class AclVenc(object):
    def __init__(self, width, height):
        self._width = width
        self._height = height
        self._size = int(width * height * 3 // 2)
        self.venc_channel_desc = 0
        self.dvpp_pic_desc = 0
        self.frame_config = None
        self.cb_thread_id = None
        self.context = None
        self.callback_run_flag = True
        self.buffer_queue = Queue()
        self._run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)

    def __del__(self):
        if self.venc_channel_desc != 0:
            acl.media.venc_destroy_channel(self.venc_channel_desc)
            acl.media.venc_destroy_channel_desc(self.venc_channel_desc)
        if self.dvpp_pic_desc != 0:
            acl.media.dvpp_destroy_pic_desc(self.dvpp_pic_desc)
        if self.frame_config:
            acl.media.venc_destroy_frame_config(self.frame_config)
        #acl.rt.destroy_context(self.context)

    def init_resource(self):
        self.context, ret = acl.rt.get_context(DEVICE_ID)
        #self.context, ret = acl.rt.get_context()
        if (self.context is None) or (ret != ACL_ERROR_NONE):
            print("acl.rt.create_context", ret)
            return FAILED
        print("Create venc context ok")

        self.cb_thread_id, ret = acl.util.start_thread(
            self.cb_thread_func, [self.context, SUBSCRIBE_TIMEOUT])
        if ret != ACL_ERROR_NONE:
            print("Start venc subscribe thread failed, error ", ret)
            return FAILED
        print("Create venc subscribe thread ok")
        
        self.frame_config = acl.media.venc_create_frame_config()
        if self.frame_config is None:
            print("Create venc frame config failed")
        print("Create venc frame config ok")
        self.venc_set_frame_config(self.frame_config, 0, 0)
        self.venc_get_frame_config(self.frame_config)

        self.venc_channel_desc = acl.media.venc_create_channel_desc()
        if self.venc_channel_desc is None:
            print("acl.media.venc_create_channel_desc failed")
            return FAILED
        print("Create venc channel desc ok")
        
        self.set_channel_desc()
        print("Set venc channel desc ok")

        ret = acl.media.venc_create_channel(self.venc_channel_desc)
        if ret != ACL_ERROR_NONE:
            print("Create venc channel failed, error ", ret)
            return FAILED
        print("Create venc channel ok")

        self.dvpp_pic_desc = acl.media.dvpp_create_pic_desc()
        ret = acl.media.dvpp_set_pic_desc_size(self.dvpp_pic_desc, self._size)
        print("Set pic desc size ok")

        return SUCCESS

    def set_channel_desc(self):
        # venc_channel_desc set function
        acl.media.venc_set_channel_desc_thread_id(
            self.venc_channel_desc, self.cb_thread_id)

        acl.media.venc_set_channel_desc_callback(
            self.venc_channel_desc, self.callback_func)
        acl.media.venc_set_channel_desc_entype(
            self.venc_channel_desc, ENTYPE_H264_MAIN)
        acl.media.venc_set_channel_desc_pic_format(
            self.venc_channel_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        key_frame_interval = 16
        acl.media.venc_set_channel_desc_key_frame_interval(
            self.venc_channel_desc, key_frame_interval)
        acl.media.venc_set_channel_desc_pic_height(
            self.venc_channel_desc, self._height)
        acl.media.venc_set_channel_desc_pic_width(
            self.venc_channel_desc, self._width)

    def cb_thread_func(self, args_list):
        context = args_list[0]
        timeout = args_list[1]
        print("[INFO] cb_thread_func args_list = ", context, timeout,
              self.callback_run_flag)

        context, ret = acl.rt.create_context(DEVICE_ID)
        if ret != 0:
            print("[INFO] cb_thread_func acl.rt.create_context ret=", ret)
            return
        while self.callback_run_flag is True:
            ret = acl.rt.process_report(timeout)

        ret = acl.rt.destroy_context(context)
        print("[INFO] cb_thread_func acl.rt.destroy_context ret=", ret)

    def callback_func(self, input_pic_desc, output_stream_desc, user_data):
        print("execute call back")
        if output_stream_desc == 0:
            print("[INFO] [venc] output_stream_desc is null")
            return
        stream_data = acl.media.dvpp_get_stream_desc_data(output_stream_desc)
        if stream_data is None:
            print("[INFO] [venc] acl.media.dvpp_get_stream_desc_data is none")
            return
        '''
        print("pop origin image")
        data = user_data.get_nowait()
        if data is not None:
            data = None
        print("release origin image ok")
        '''
        ret = acl.media.dvpp_get_stream_desc_ret_code(output_stream_desc)
        print("ret code ", ret)
        if ret == 0:
            stream_data_size = acl.media.dvpp_get_stream_desc_size(
                output_stream_desc)
            print("[INFO] [venc] stream_data size", stream_data_size)
            # stream memcpy d2h
            np_data = acl.util.ptr_to_numpy(stream_data, (stream_data_size, ), NPY_BYTE)
            with open('./output.h265', 'ab') as f:
                print("save venc file")
                f.write(np_data)
        else:
            print("callback failed")

    def venc_set_frame_config(self, frame_confg, eos, iframe):
        acl.media.venc_set_frame_config_eos(frame_confg, eos)
        acl.media.venc_set_frame_config_force_i_frame(frame_confg, iframe)

    def venc_get_frame_config(self, frame_confg):
        get_eos = acl.media.venc_get_frame_config_eos(frame_confg)
        if get_eos != ACL_ERROR_NONE:
            print("Venc get frame config failed")
            return FAILED

        get_force_frame = acl.media.venc_get_frame_config_force_i_frame(
            frame_confg)
        if get_force_frame != ACL_ERROR_NONE:
            print("Venc get frame config force I frame failed")
            return FAILED
        return SUCCESS

    def process(self, image, is_finished=False):
        ret = SUCCESS
        if is_finished == False:
            print("set pic desc data")
            #self.buffer_queue.put(image)
            acl.media.dvpp_set_pic_desc_data(self.dvpp_pic_desc, image.data())
            print("send venc frame, data ", image.data())
            ret = acl.media.venc_send_frame(self.venc_channel_desc,                
                                            self.dvpp_pic_desc, 0, 
                                            self.frame_config, self.buffer_queue)
            if ret != ACL_ERROR_NONE:
                print("Venc send frame failed, error ", ret)
            else:
                print("send venc frame ok")
        else:
            self.venc_set_frame_config(self.frame_config, 1, 0)
            # send eos frame
            print("[INFO] venc send frame eos")
            ret = acl.media.venc_send_frame(
                self.venc_channel_desc, 0, 0, self.frame_config, None)
            if ret != ACL_ERROR_NONE:
                print("Venc send eos frame failed, error ", ret)
            self.thread_join()

        return ret

    def thread_join(self):
        self.callback_run_flag = False
        ret = acl.util.stop_thread(self.cb_thread_id)
        print("[INFO] stop_thread", ret)

    def venc_stream_desc_set(self):
        stream_format = 0
        timestamp = 123456
        ret_code = 1
        eos = 1

        # stream desc
        dvpp_stream_desc = acl.media.dvpp_create_stream_desc()
        if dvpp_stream_desc is None:
            print("Create venc stream desc failed")
            return FAILED

        # stream_desc set function
        acl.media.dvpp_set_stream_desc_format(dvpp_stream_desc, stream_format)
        acl.media.dvpp_set_stream_desc_timestamp(dvpp_stream_desc, timestamp)
        acl.media.dvpp_set_stream_desc_ret_code(dvpp_stream_desc, ret_code)
        acl.media.dvpp_set_stream_desc_eos(dvpp_stream_desc, eos)

        ret = acl.media.dvpp_destroy_stream_desc(dvpp_stream_desc)
        if ret != ACL_ERROR_NONE:
            print("Destroy stream desc failed")
            return FAILED

        return SUCCESS




def main():
    acl.init()
    acl.rt.set_device(0)

    dvpp_venc = AclVenc(1280, 720)
    ret = dvpp_venc.init_resource()
    if ret != SUCCESS:
        print("Init venc failed")
        return

if __name__ == '__main__':
    main()