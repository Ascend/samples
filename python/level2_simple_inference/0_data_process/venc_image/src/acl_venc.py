# -*- coding:utf-8 -*-
# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import numpy as np
import acl

# enum
memcpy_kind = {
    "ACL_MEMCPY_HOST_TO_HOST": 0,
    "ACL_MEMCPY_HOST_TO_DEVICE": 1,
    "ACL_MEMCPY_DEVICE_TO_HOST": 2,
    "ACL_MEMCPY_DEVICE_TO_DEVICE": 3
}

"""
enType 0
0 H265 main level
1 H264 baseline level
2 H264 main level
3 H264 high level
"""
VENC_ENTYPE = 0
"""
format 2
1 YUV420 semi-planner (nv12)
2 YVU420 semi-planner (nv21)
"""
VENC_FORMAT = 1

DEVICE_ID = 0
IMAGE_INPUT_WIDTH = 128
IMAGE_INPUT_HEIGHT = 128

VENC_FILE_PATH = "../data/dvpp_venc_128x128_nv12.yuv"


def check_ret(message, ret_int):
    """
    功能简介：检测pyACL函数返回值是否正常，如果非0则会抛出异常
    参数：ret，pyACL函数返回值
    返回值：无
    """
    if ret_int != 0:
        raise Exception("{} failed ret_int={}"
                        .format(message, ret_int))


def check_none(message, ret_none):
    """
    功能简介：检测值是否为空，如果为空则会抛出异常
    参数：ret_none
    返回值：无
    """
    if ret_none is None:
        raise Exception("{} failed"
                        .format(message))


class AclVenc(object):
    """
    视频编码
    """
    def __init__(self):
        self.input_stream_mem = None
        self.venc_channel_desc = 0
        self.dvpp_pic_desc = 0
        self.frame_config = None
        self.cb_thread_id = None
        self.pic_host = None
        self.context, ret = acl.rt.create_context(DEVICE_ID)
        check_ret("acl.rt.create_context", ret)
        self.callback_run_flag = True
        self.stream_host = None
        if os.path.exists('./data/output.h265'):
            os.remove('./data/output.h265')

    def release_resource(self):
        print("[INFO] free resource")
        if self.stream_host:
            acl.rt.free_host(self.stream_host)
        if self.pic_host:
            acl.rt.free_host(self.pic_host)
        if self.input_stream_mem:
            acl.media.dvpp_free(self.input_stream_mem)
        if self.venc_channel_desc != 0:
            acl.media.venc_destroy_channel_desc(self.venc_channel_desc)
        if self.dvpp_pic_desc != 0:
            acl.media.dvpp_destroy_pic_desc(self.dvpp_pic_desc)
        if self.frame_config:
            acl.media.venc_destroy_frame_config(self.frame_config)
        acl.rt.destroy_context(self.context)

    def venc_init(self):
        self.venc_channel_desc = acl.media.venc_create_channel_desc()
        check_none("acl.media.venc_create_channel_desc",
                   self.venc_channel_desc)

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
        if output_stream_desc == 0:
            print("[INFO] [venc] output_stream_desc is null")
            return
        stream_data = acl.media.dvpp_get_stream_desc_data(output_stream_desc)
        if stream_data is None:
            print("[INFO] [venc] acl.media.dvpp_get_stream_desc_data is none")
            return
        ret = acl.media.dvpp_get_stream_desc_ret_code(output_stream_desc)
        if ret == 0:
            stream_data_size = acl.media.dvpp_get_stream_desc_size(
                output_stream_desc)
            print("[INFO] [venc] stream_data size", stream_data_size)
            # stream memcpy d2h
            if "bytes_to_ptr" in dir(acl.util):
                data = bytes(stream_data_size)
                data_ptr = acl.util.bytes_to_ptr(data)
            else:
                data = np.zeros(stream_data_size, dtype=np.byte)
                data_ptr = acl.util.numpy_to_ptr(data)
            ret = acl.rt.memcpy(data_ptr, stream_data_size,
                                stream_data, stream_data_size,
                                memcpy_kind.get("ACL_MEMCPY_DEVICE_TO_HOST"))
            if ret != 0:
                print("[INFO] [venc] acl.rt.memcpy ret=", ret)
                return
            current_dir = os.path.dirname(os.path.abspath(__file__))
            output_path = os.path.join(current_dir, '../data/output.h265')
            with open(output_path, 'ab') as f:
                f.write(data)

    def venc_set_desc(self, width, height):
        # venc_channel_desc set function
        acl.media.venc_set_channel_desc_thread_id(
            self.venc_channel_desc, self.cb_thread_id)

        acl.media.venc_set_channel_desc_callback(
            self.venc_channel_desc, self.callback_func)
        acl.media.venc_set_channel_desc_entype(
            self.venc_channel_desc, VENC_ENTYPE)
        acl.media.venc_set_channel_desc_pic_format(
            self.venc_channel_desc, VENC_FORMAT)
        key_frame_interval = 16
        acl.media.venc_set_channel_desc_key_frame_interval(
            self.venc_channel_desc, key_frame_interval)
        acl.media.venc_set_channel_desc_pic_height(
            self.venc_channel_desc, height)
        acl.media.venc_set_channel_desc_pic_width(
            self.venc_channel_desc, width)

    def venc_set_frame_config(self, frame_confg, eos, iframe):
        acl.media.venc_set_frame_config_eos(frame_confg, eos)
        acl.media.venc_set_frame_config_force_i_frame(frame_confg, iframe)

    def venc_get_frame_config(self, frame_confg):
        get_eos = acl.media.venc_get_frame_config_eos(frame_confg)
        check_ret("acl.media.venc_get_frame_config_eos", get_eos)
        get_force_frame = acl.media.venc_get_frame_config_force_i_frame(
            frame_confg)
        check_ret("acl.media.venc_get_frame_config_force_i_frame",
                  get_force_frame)

    def venc_process(self, input_mem, input_size):
        # set picture description
        self.dvpp_pic_desc = acl.media.dvpp_create_pic_desc()
        check_none("acl.media.dvpp_create_pic_desc", self.dvpp_pic_desc)
        ret = acl.media.dvpp_set_pic_desc_data(self.dvpp_pic_desc, input_mem)
        ret = acl.media.dvpp_set_pic_desc_size(self.dvpp_pic_desc, input_size)
        print("[INFO] set pic desc size")

        self.venc_set_frame_config(self.frame_config, 0, 0)
        print("[INFO] set frame config")
        self.venc_get_frame_config(self.frame_config)

        # send frame
        venc_cnt = 16
        while venc_cnt:
            ret = acl.media.venc_send_frame(
                self.venc_channel_desc,
                self.dvpp_pic_desc, 0, self.frame_config, None)
            check_ret("acl.media.venc_send_frame", ret)
            venc_cnt -= 1

        self.venc_set_frame_config(self.frame_config, 1, 0)

        # send eos frame
        print("[INFO] venc send frame eos")
        ret = acl.media.venc_send_frame(
            self.venc_channel_desc, 0, 0, self.frame_config, None)
        print("[INFO] acl.media.venc_send_frame ret=", ret)

        print("[INFO] venc process success")

    def venc_run(self):
        timeout = 1000
        self.cb_thread_id, ret = acl.util.start_thread(
            self.cb_thread_func, [self.context, timeout])
        check_ret("acl.util.start_thread", ret)
        print("[INFO] start_thread", self.cb_thread_id, ret)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        # load file
        file_context = np.fromfile(os.path.join(current_dir, VENC_FILE_PATH),
                                   dtype=np.byte)
        file_size = file_context.size
        if "bytes_to_ptr" in dir(acl.util):
            bytes_data = file_context.tobytes()
            file_mem = acl.util.bytes_to_ptr(bytes_data)
        else:
            file_mem = acl.util.numpy_to_ptr(file_context)

        input_size = file_size
        input_mem, ret = acl.media.dvpp_malloc(input_size)
        check_ret("acl.media.dvpp_malloc", ret)

        ret = acl.rt.memcpy(
            input_mem, input_size, file_mem,
            file_size, memcpy_kind.get("ACL_MEMCPY_HOST_TO_DEVICE"))
        check_ret("acl.rt.memcpy", ret)

        self.venc_set_desc(IMAGE_INPUT_WIDTH, IMAGE_INPUT_HEIGHT)
        print("[INFO] set venc channel desc")

        ret = acl.media.venc_create_channel(self.venc_channel_desc)
        check_ret("acl.media.venc_create_channel", ret)

        self.frame_config = acl.media.venc_create_frame_config()
        check_none("acl.media.venc_create_frame_config", self.frame_config)
        print("[INFO] create_frame_config")

        self.venc_process(input_mem, input_size)
        print("[INFO] venc_process end")

        ret = acl.media.dvpp_free(input_mem)
        check_ret("acl.media.dvpp_free", ret)

        ret = acl.media.venc_destroy_channel(self.venc_channel_desc)
        check_ret("acl.media.venc_destroy_channel", ret)

        self.thread_join()
        print("[INFO] thread join")

    def venc_stream_desc_set(self):
        stream_format = 0
        timestamp = 123456
        ret_code = 1
        eos = 1

        # stream desc
        dvpp_stream_desc = acl.media.dvpp_create_stream_desc()
        check_none("acl.media.dvpp_create_stream_desc", dvpp_stream_desc)

        # stream_desc set function
        acl.media.dvpp_set_stream_desc_format(dvpp_stream_desc, stream_format)
        acl.media.dvpp_set_stream_desc_timestamp(dvpp_stream_desc, timestamp)
        acl.media.dvpp_set_stream_desc_ret_code(dvpp_stream_desc, ret_code)
        acl.media.dvpp_set_stream_desc_eos(dvpp_stream_desc, eos)

        ret = acl.media.dvpp_destroy_stream_desc(dvpp_stream_desc)
        check_ret("acl.media.dvpp_destroy_stream_desc", ret)

    def thread_join(self):
        self.callback_run_flag = False
        ret = acl.util.stop_thread(self.cb_thread_id)
        print("[INFO] stop_thread", ret)


if __name__ == '__main__':
    ret = acl.init("")
    check_ret("acl.init", ret)
    ret = acl.rt.set_device(DEVICE_ID)
    check_ret("acl.rt.set_device", ret)
    run_mode, ret = acl.rt.get_run_mode()
    check_ret("acl.rt.get_run_mode", ret)
    venc_handle = AclVenc()
    venc_handle.venc_init()
    venc_handle.venc_run()
    venc_handle.venc_stream_desc_set()
    venc_handle.release_resource()
    ret = acl.rt.reset_device(DEVICE_ID)
    check_ret("acl.rt.reset_device", ret)
    ret = acl.finalize()
    check_ret("acl.finalize", ret)
