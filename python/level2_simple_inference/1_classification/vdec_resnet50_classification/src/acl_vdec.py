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
import numpy as np
import acl
from constant import ACL_MEMCPY_HOST_TO_DEVICE, \
    H265_MAIN_LEVEL, PIXEL_FORMAT_YVU_SEMIPLANAR_420
from acl_util import check_ret


class Vdec(object):
    def __init__(self, context, stream, vdec_out_path):
        self.context = context
        self.stream = stream
        self.vdec_out_path = vdec_out_path
        self.vdec_channel_desc = None
        self.input_width = 0
        self.input_height = 0
        self._vdec_exit = True
        self._en_type = H265_MAIN_LEVEL
        self._format = PIXEL_FORMAT_YVU_SEMIPLANAR_420
        self._channel_id = 10
        self.output_count = 1
        self.rest_len = 10
        self.images_buffer = []

    def __del__(self):
        print("[Vdec] release resource:")
        ret = acl.media.dvpp_free(self.input_stream_mem)
        check_ret("acl.media.dvpp_free", ret)
        ret = acl.media.vdec_destroy_channel_desc(self.vdec_channel_desc)
        check_ret("acl.media.vdec_destroy_channel_desc", ret)
        ret = acl.media.vdec_destroy_frame_config(self.frame_config)
        check_ret("acl.media.vdec_destroy_frame_config", ret)
        print("[Vdec] release resource success")

    def _thread_func(self, args_list):
        timeout = args_list[0]
        acl.rt.set_context(self.context)
        while self._vdec_exit:
            acl.rt.process_report(timeout)
        print("[Vdec] [_thread_func] _thread_func out")

    def _callback(self, input_stream_desc, output_pic_desc, user_data):
        # input_stream_desc
        if input_stream_desc:
            ret = acl.media.dvpp_destroy_stream_desc(input_stream_desc)
            if ret != 0:
                print("acl.media.dvpp_destroy_stream_desc failed")
        # output_pic_desc
        if output_pic_desc:
            vdec_out_buffer = acl.media.dvpp_get_pic_desc_data(output_pic_desc)
            acl.media.dvpp_get_pic_desc_ret_code(output_pic_desc)
            '''
            here no need to copy back to host, because, the data will send to infer process.
            '''
            data_size = acl.media.dvpp_get_pic_desc_size(output_pic_desc)
            print("data_size in call_back:", data_size)
            self.images_buffer.append(dict({"buffer": vdec_out_buffer,
                                            "size": data_size}))
            ret = acl.media.dvpp_destroy_pic_desc(output_pic_desc)
            if ret != 0:
                print("acl.media.dvpp_destroy_pic_desc failed")
        self.output_count += 1
        print("[Vdec] [_callback] _callback exit success")

    def get_image_buffer(self):
        return self.images_buffer

    def init_resource(self, cb_thread_id):
        print("[Vdec] class Vdec init resource stage:")
        self.vdec_channel_desc = acl.media.vdec_create_channel_desc()
        acl.media.vdec_set_channel_desc_channel_id(self.vdec_channel_desc,
                                                   self._channel_id)
        acl.media.vdec_set_channel_desc_thread_id(self.vdec_channel_desc,
                                                  cb_thread_id)
        acl.media.vdec_set_channel_desc_callback(self.vdec_channel_desc,
                                                 self._callback)
        acl.media.vdec_set_channel_desc_entype(self.vdec_channel_desc,
                                               self._en_type)
        acl.media.vdec_set_channel_desc_out_pic_format(self.vdec_channel_desc,
                                                       self._format)
        acl.media.vdec_create_channel(self.vdec_channel_desc)
        print("[Vdec] class Vdec init resource stage success")

    def _gen_input_dataset(self, img_path):
        img = np.fromfile(img_path, dtype=self.dtype)
        img_buffer_size = img.size
        if "bytes_to_ptr" in dir(acl.util):
            bytes_data = img.tobytes()
            img_ptr = acl.util.bytes_to_ptr(bytes_data)
        else:
            img_ptr = acl.util.numpy_to_ptr(img)
        img_device, ret = acl.media.dvpp_malloc(img_buffer_size)
        ret = acl.rt.memcpy(img_device,
                            img_buffer_size,
                            img_ptr,
                            img_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)
        return img_device, img_buffer_size

    def _set_input(self, input_stream_size):
        self.dvpp_stream_desc = acl.media.dvpp_create_stream_desc()
        ret = acl.media.dvpp_set_stream_desc_data(self.dvpp_stream_desc,
                                                  self.input_stream_mem)
        check_ret("acl.media.dvpp_set_stream_desc_data", ret)
        ret = acl.media.dvpp_set_stream_desc_size(self.dvpp_stream_desc,
                                                  input_stream_size)
        check_ret("acl.media.dvpp_set_stream_desc_size", ret)
        print("[Vdec] create input stream desc success")

    def _set_pic_output(self, output_pic_size):
        # pic_desc
        output_pic_mem, ret = acl.media.dvpp_malloc(output_pic_size)
        check_ret("acl.media.dvpp_malloc", ret)

        self.dvpp_pic_desc = acl.media.dvpp_create_pic_desc()
        

        acl.media.dvpp_set_pic_desc_height(self.dvpp_pic_desc, self.input_height)
        acl.media.dvpp_set_pic_desc_width(self.dvpp_pic_desc, self.input_width)
        
        vdec_out_height = int(int((self.input_height + 1) / 2) * 2)
        vdec_out_width = int(int((self.input_width + 15) / 16) * 16)
        acl.media.dvpp_set_pic_desc_width_stride(self.dvpp_pic_desc, vdec_out_width)
        acl.media.dvpp_set_pic_desc_height_stride(self.dvpp_pic_desc, vdec_out_height)
        
        
        acl.media.dvpp_set_pic_desc_data(self.dvpp_pic_desc,
                                         output_pic_mem)

        acl.media.dvpp_set_pic_desc_size(self.dvpp_pic_desc,
                                         output_pic_size)

        acl.media.dvpp_set_pic_desc_format(self.dvpp_pic_desc,
                                           self._format)
        print("[Vdec] create output pic desc success")

    def forward(self, output_pic_size, input_stream_size):
        self.frame_config = acl.media.vdec_create_frame_config()

        for i in range(self.rest_len):
            print("[Vdec] forward index:{}".format(i))
            self._set_input(input_stream_size)           

            self._set_pic_output(output_pic_size)

            # vdec_send_frame
            ret = acl.media.vdec_send_frame(self.vdec_channel_desc,
                                            self.dvpp_stream_desc,
                                            self.dvpp_pic_desc,
                                            self.frame_config,
                                            None)
            check_ret("acl.media.vdec_send_frame", ret)
            print('[Vdec] vdec_send_frame stage success')

    def run(self, video_path):
        self.video_path, self.input_width, self.input_height, \
            self.dtype = video_path
        # here set callback timeout time.
        timeout = 100
        cb_thread_id, ret = acl.util.start_thread(
            self._thread_func, [timeout])

        self.init_resource(cb_thread_id)              
        
        # vdec output need to be stride to 16*2
        vdec_out_height = int(int((self.input_height + 1) / 2) * 2)
        vdec_out_width = int(int((self.input_width + 15) / 16) * 16)
        output_pic_size = vdec_out_width * vdec_out_height * 3 // 2
                
        
        
        # input_stream_size: is the size read from original stream file.
        # input_stream_mem：the ptr to the data, which have been copy to device after read from original file.
        self.input_stream_mem, input_stream_size = self. \
            _gen_input_dataset(self.video_path)
        
 
        self.forward(output_pic_size, input_stream_size)

        ret = acl.media.vdec_destroy_channel(self.vdec_channel_desc)
        check_ret("acl.media.vdec_destroy_channel", ret)

        self._vdec_exit = False
        ret = acl.util.stop_thread(cb_thread_id)
        check_ret("acl.util.stop_thread", ret)
        print("[Vdec] vdec finish!!!\n")

    def _destroy_resource(self):
        print("[Vdec] release resource:")
        ret = acl.media.dvpp_free(self.input_stream_mem)
        check_ret("acl.media.dvpp_free", ret)
        ret = acl.media.vdec_destroy_channel(self.vdec_channel_desc)
        check_ret("acl.media.vdec_destroy_channel", ret)
        ret = acl.media.vdec_destroy_channel_desc(self.vdec_channel_desc)
        check_ret("acl.media.vdec_destroy_channel_desc", ret)
        ret = acl.media.vdec_destroy_frame_config(self.frame_config)
        check_ret("acl.media.vdec_destroy_frame_config", ret)
        print("[Vdec] release resource success")
