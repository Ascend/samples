"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-8-19 08:12:13
MODIFIED: 2020-8-19 21:04:45
"""
import time
import os
import numpy as np
import acl

from atlas_utils.acl_image import AclImage
from atlas_utils import utils
from atlas_utils.acl_logger import log_info, log_error
import atlas_utils.constants as const


class AclVenc(object):
    def __init__(self, width, height, filename,
                 input_format=const.PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                 out_entype=const.ENTYPE_H264_MAIN):
        self._width = width
        self._height = height
        self._filename = filename
        self._input_format = input_format
        self._out_entype = out_entype
        self.venc_channel_desc = 0
        self.dvpp_pic_desc = 0
        self.frame_config = None
        self._cb_thread_id = None
        self._is_thread_exit = False
        self.callback_run_flag = True
        self._is_inited = False
        self._is_fin = False
        self._is_destroyed = False
        self._init_resource()

    def __del__(self):
        self.destroy()

    def destroy(self):
        if _is_destroyed:
            return

        self.finish()

        if self.venc_channel_desc != 0:
            acl.media.venc_destroy_channel(self.venc_channel_desc)
            acl.media.venc_destroy_channel_desc(self.venc_channel_desc)
        if self.dvpp_pic_desc != 0:
            acl.media.dvpp_destroy_pic_desc(self.dvpp_pic_desc)
        if self.frame_config:
            acl.media.venc_destroy_frame_config(self.frame_config)

        self._is_destroyed = True

    def _init_resource(self):
        if self._is_inited:
            return

        self.venc_channel_desc = acl.media.venc_create_channel_desc()
        utils.check_none("acl.media.venc_create_channel_desc",
                   self.venc_channel_desc)
        self._cb_thread_id, ret = acl.util.start_thread(self._cb_thread_func, [])            
        utils.check_ret("acl.util.start_thread", ret)
        log_info("[INFO] start_thread", self._cb_thread_id, ret)

        self._set_channel_desc()
        log_info("[INFO] set venc channel desc")

        ret = acl.media.venc_create_channel(self.venc_channel_desc)
        utils.check_ret("acl.media.venc_create_channel", ret)

        self.frame_config = acl.media.venc_create_frame_config()
        utils.check_none("acl.media.venc_create_frame_config", self.frame_config)
        log_info("[INFO] create_frame_config")
                # set picture description
        self.dvpp_pic_desc = acl.media.dvpp_create_pic_desc()
        utils.check_none("acl.media.dvpp_create_pic_desc", self.dvpp_pic_desc)

        self._set_frame_config(self.frame_config, 0, 0)
        log_info("[INFO] set frame config")
        self._get_frame_config(self.frame_config)

        self._is_inited = True

    def _cb_thread_func(self, args_list):
        context, ret = acl.rt.create_context(0)
        if ret != 0:
            log_info("[INFO] cb_thread_func acl.rt.create_context ret=", ret)
            return
        while self.callback_run_flag is True:
            ret = acl.rt.process_report(1000)

        ret = acl.rt.destroy_context(context)
        log_info("[INFO] cb_thread_func acl.rt.destroy_context ret=", ret)
        self._is_thread_exit = True

    def _callback_func(self, input_pic_desc, output_stream_desc, user_data):
        if output_stream_desc == 0:
            log_info("[INFO] [venc] output_stream_desc is null")
            return
        stream_data = acl.media.dvpp_get_stream_desc_data(output_stream_desc)
        if stream_data is None:
            log_info("[INFO] [venc] acl.media.dvpp_get_stream_desc_data is none")
            return
        ret = acl.media.dvpp_get_stream_desc_ret_code(output_stream_desc)
        if ret == 0:
            stream_data_size = acl.media.dvpp_get_stream_desc_size(
                output_stream_desc)
            log_info("[INFO] [venc] stream_data size", stream_data_size)
            # stream memcpy d2h
            np_data = np.zeros(stream_data_size, dtype=np.byte)
            np_data_ptr = acl.util.numpy_to_ptr(np_data)
            ret = acl.rt.memcpy(np_data_ptr, stream_data_size,
                                stream_data, stream_data_size,
                                const.ACL_MEMCPY_DEVICE_TO_DEVICE)
            with open(self._filename, 'ab') as f:
                f.write(np_data)
            
    def _set_channel_desc(self):
        # venc_channel_desc set function
        acl.media.venc_set_channel_desc_thread_id(
            self.venc_channel_desc, self._cb_thread_id)

        acl.media.venc_set_channel_desc_callback(
            self.venc_channel_desc, self._callback_func)
        acl.media.venc_set_channel_desc_entype(
            self.venc_channel_desc, self._out_entype)
        acl.media.venc_set_channel_desc_pic_format(
            self.venc_channel_desc, self._input_format)
        key_frame_interval = 16
        acl.media.venc_set_channel_desc_key_frame_interval(
            self.venc_channel_desc, key_frame_interval)
        acl.media.venc_set_channel_desc_pic_height(
            self.venc_channel_desc, self._height)
        acl.media.venc_set_channel_desc_pic_width(
            self.venc_channel_desc, self._width)

    def _set_frame_config(self, frame_confg, eos, iframe):
        acl.media.venc_set_frame_config_eos(frame_confg, eos)
        acl.media.venc_set_frame_config_force_i_frame(frame_confg, iframe)

    def _get_frame_config(self, frame_confg):
        get_eos = acl.media.venc_get_frame_config_eos(frame_confg)
        utils.check_ret("acl.media.venc_get_frame_config_eos", get_eos)
        get_force_frame = acl.media.venc_get_frame_config_force_i_frame(
            frame_confg)
        utils.check_ret("acl.media.venc_get_frame_config_force_i_frame",
                  get_force_frame)

    def process(self, image):
        ret = acl.media.dvpp_set_pic_desc_data(self.dvpp_pic_desc, image.data())
        ret = acl.media.dvpp_set_pic_desc_size(self.dvpp_pic_desc, image.size)
        log_info("[INFO] set pic desc size")

        # send frame

        ret = acl.media.venc_send_frame(
            self.venc_channel_desc,
            self.dvpp_pic_desc, 0, self.frame_config, None)
        utils.check_ret("acl.media.venc_send_frame", ret)

    
    def finish(self):
        if self._is_fin:
            return

        self._set_frame_config(self.frame_config, 1, 0)

        # send eos frame
        log_info("[INFO] venc send frame eos")
        ret = acl.media.venc_send_frame(
            self.venc_channel_desc, 0, 0, self.frame_config, None)
        log_info("[INFO] acl.media.venc_send_frame ret=", ret)
        self._set_eos_stream_desc()
        self._thread_join()
        self._is_fin = True

    def _set_eos_stream_desc(self):
        stream_format = 0
        timestamp = 123456
        ret_code = 1
        eos = 1

        # stream desc
        dvpp_stream_desc = acl.media.dvpp_create_stream_desc()
        utils.check_none("acl.media.dvpp_create_stream_desc", dvpp_stream_desc)

        # stream_desc set function
        acl.media.dvpp_set_stream_desc_format(dvpp_stream_desc, stream_format)
        acl.media.dvpp_set_stream_desc_timestamp(dvpp_stream_desc, timestamp)
        acl.media.dvpp_set_stream_desc_ret_code(dvpp_stream_desc, ret_code)
        acl.media.dvpp_set_stream_desc_eos(dvpp_stream_desc, eos)

        ret = acl.media.dvpp_destroy_stream_desc(dvpp_stream_desc)
        utils.check_ret("acl.media.dvpp_destroy_stream_desc", ret)

    def _thread_join(self):
        self.callback_run_flag = False
        while self._is_thread_exit == False:
            time.sleep(0.01)
        ret = acl.util.stop_thread(self._cb_thread_id)
        log_info("[INFO] stop_thread", ret)


if __name__ == '__main__':
    ret = acl.init("")
    utils.check_ret("acl.init", ret)
    ret = acl.rt.set_device(DEVICE_ID)
    utils.check_ret("acl.rt.set_device", ret)
    run_mode, ret = acl.rt.get_run_mode()
    utils.check_ret("acl.rt.get_run_mode", ret)
    venc_handel = AclVenc()

    venc_cnt = 16
    while venc_cnt:
        # load file
        image = AclImage(VENC_FILE_PATH, 1280, 720)
        image = image.copy_to_dvpp()
        venc_handel.process(image)
        venc_cnt -= 1



    log_info("process end") 
    venc_handel.finish()
    ret = acl.rt.reset_device(DEVICE_ID)
    utils.check_ret("acl.rt.reset_device", ret)
    ret = acl.finalize()
    utils.check_ret("acl.finalize", ret)
