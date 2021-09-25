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
import acl
from constant import PIXEL_FORMAT_YUV_SEMIPLANAR_420
from acl_util import check_ret


class Dvpp(object):
    """
    图片解码
    """
    def __init__(self, stream, model_input_width, model_input_height):
        self._dvpp_channel_desc = None
        self._resize_config = None
        self._decode_out_dev_buffer = None
        self._decode_output_desc_ = None
        self._resize_input_desc_ = None
        self._resize_out_desc = None
        self._in_dev_buffer_ = None
        self._in_dev_buffer_size = 0
        self._input_width = 0
        self._input_height = 0
        self._resize_out_dev = None
        self._resize_out_size = 0
        self.stream = stream
        self._format = PIXEL_FORMAT_YUV_SEMIPLANAR_420
        self._model_input_width = model_input_width
        self._model_input_height = model_input_height

        self.init_resource()

    def __del__(self):
        if self._decode_out_dev_buffer:
            acl.media.dvpp_free(self._decode_out_dev_buffer)
            self._decode_out_dev_buffer = None

        if self._resize_input_desc_:
            acl.media.dvpp_destroy_pic_desc(self._resize_input_desc_)
            self._resize_input_desc_ = None

        if self._resize_out_desc:
            acl.media.dvpp_destroy_pic_desc(self._resize_out_desc)
            self._resize_out_desc = None

        if self._resize_config:
            acl.media.dvpp_destroy_resize_config(self._resize_config)
            self._resize_config = None

        if self._dvpp_channel_desc:
            acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)
            self._dvpp_channel_desc = None

        if self._resize_out_dev:
            acl.media.dvpp_free(self._resize_out_dev)
            self._resize_out_dev = None
        print("[Dvpp] class Dvpp exit success")

    def init_resource(self):
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        self._resize_config = acl.media.dvpp_create_resize_config()

    def get_output(self):
        return self._resize_out_dev, self._resize_out_size

    def gen_tensor_desc(self,
                        temp_buffer,
                        temp_width,
                        temp_height,
                        flag=True,
                        need_malloc=True):
        if flag:
            decode_out_width = ((temp_width + 127) // 128) * 128
            decode_out_height = ((temp_height + 15) // 16) * 16
        else:
            if temp_height % 2 or temp_width % 2:
                raise Exception("[Dvpp] output width={} or height={} is odd"
                                .format(temp_width, temp_height))
            decode_out_width = temp_width
            decode_out_height = temp_height

        decode_out_buffer_size = decode_out_width * decode_out_height * 3 // 2

        if need_malloc:
            temp_buffer, result = acl.media.dvpp_malloc(decode_out_buffer_size)
            check_ret("acl.media.dvpp_malloc", result)

        temp_desc = acl.media.dvpp_create_pic_desc()
        # 设置图片描述的属性参数
        acl.media.dvpp_set_pic_desc_data(temp_desc, temp_buffer)
        acl.media.dvpp_set_pic_desc_format(temp_desc, self._format)
        acl.media.dvpp_set_pic_desc_width(temp_desc, temp_width)
        acl.media.dvpp_set_pic_desc_height(temp_desc, temp_height)
        acl.media.dvpp_set_pic_desc_width_stride(temp_desc, decode_out_width)
        acl.media.dvpp_set_pic_desc_height_stride(temp_desc, decode_out_height)
        acl.media.dvpp_set_pic_desc_size(temp_desc, decode_out_buffer_size)
        return temp_desc, temp_buffer, decode_out_buffer_size

    def _decode_process(self,
                        img_buffer,
                        img_buffer_size,
                        img_width,
                        img_height):
        print('[Dvpp] vpc decode stage:')
        self._decode_output_desc_, self._decode_out_dev_buffer, temp_size = \
            self.gen_tensor_desc(self._decode_out_dev_buffer,
                                 img_width, img_height)
        ret = acl.media.dvpp_jpeg_decode_async(self._dvpp_channel_desc,
                                               img_buffer,
                                               img_buffer_size,
                                               self._decode_output_desc_,
                                               self.stream)
        if ret != 0:
            raise Exception("dvpp_jpeg_decode_async failed ret={}".format(ret))
        acl.rt.synchronize_stream(self.stream)
        print('[Dvpp] vpc decode stage success')

    def _resize_process(self, img_width, img_height):
        print('[Dvpp] vpc resize stage:')
        self._resize_input_desc_, self._decode_out_buffer, _resize_in_size = \
            self.gen_tensor_desc(self._decode_out_dev_buffer,
                                 img_width,
                                 img_height,
                                 need_malloc=False)
        self._resize_out_desc, self._resize_out_dev, self._resize_out_size = \
            self.gen_tensor_desc(self._resize_out_dev,
                                 self._model_input_width,
                                 self._model_input_height,
                                 flag=False)

        ret = acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                              self._resize_input_desc_,
                                              self._resize_out_desc,
                                              self._resize_config,
                                              self.stream)
        check_ret("acl.media.dvpp_vpc_resize_async", ret)

        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] vpc resize stage success')

    def run(self,
            img_buffer,
            img_buffer_size,
            img_width,
            img_height):
        self._decode_process(img_buffer,
                             img_buffer_size,
                             img_width,
                             img_height)

        decode_out_w = acl.media.dvpp_get_pic_desc_width(self._decode_output_desc_)
        decode_out_h = acl.media.dvpp_get_pic_desc_height(self._decode_output_desc_)
        if self._decode_output_desc_:
            acl.media.dvpp_destroy_pic_desc(self._decode_output_desc_)
            self._decode_output_desc_ = None
        self._resize_process(decode_out_w, decode_out_h)
        return self._resize_out_dev, self._resize_out_size
