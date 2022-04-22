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
from acl_util import check_ret
from constant import PIXEL_FORMAT_YVU_SEMIPLANAR_420


class Dvpp(object):
    def __init__(self, stream, model_input_width, model_input_height):
        self._dvpp_channel_desc = None
        self._resize_config = None
        self._resize_in_desc_ = None
        self._resize_out_desc = None
        self._decode_out_buffer = None
        self._in_dev_buffer_ = None
        self._in_dev_buffer_size = 0
        self._input_width = 0
        self._input_height = 0
        self._format = PIXEL_FORMAT_YVU_SEMIPLANAR_420
        self._resize_out_dev = None
        self._resize_out_size = 0
        self.stream = stream
        self._model_input_width = model_input_width
        self._model_input_height = model_input_height

        self.init_resource()

    def __del__(self):
        if self._decode_out_buffer:
            acl.media.dvpp_free(self._decode_out_buffer)
            self._decode_out_buffer = None

        if self._resize_in_desc_:
            acl.media.dvpp_destroy_pic_desc(self._resize_in_desc_)
            self._resize_in_desc_ = None

        if self._resize_out_desc:
            acl.media.dvpp_destroy_pic_desc(self._resize_out_desc)
            self._resize_out_desc = None

        if self._resize_config:
            ret = acl.media.dvpp_destroy_resize_config(self._resize_config)
            check_ret("acl.media.dvpp_destroy_resize_config", ret)

        if self._dvpp_channel_desc:
            ret = acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            check_ret("acl.media.dvpp_destroy_channel", ret)
            ret = acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)
            check_ret("acl.media.dvpp_destroy_channel_desc", ret)

        if self._resize_out_dev:
            ret = acl.media.dvpp_free(self._resize_out_dev)
            check_ret("acl.media.dvpp_free", ret)
        print("[Dvpp] class Dvpp release source success")

    def init_resource(self):
        print("[Dvpp] class Dvpp init resource stage:")
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        check_ret("acl.media.dvpp_create_channel", ret)
        self._resize_config = acl.media.dvpp_create_resize_config()
        print("[Dvpp] class Dvpp init resource stage success")

    def get_output(self):
        return self._resize_out_dev, self._resize_out_size

    # set input and output desc
    def gen_tensor_desc(self,
                        temp_buffer,
                        temp_width,
                        temp_height,
                        flag=True,
                        need_malloc=True):
        if flag:
            # stride_width = int(int((temp_width + 127) / 128) * 128)
            # stride_height = int(int((temp_height + 15) / 16) * 16)
            # because here the input is from vdec, the input stride is 16*2;
            # and the output of resize is vpc constraint, the output stride is 16*2 too.
            stride_width = int(int((temp_width + 15) / 16) * 16)
            stride_height = int(int((temp_height + 1) / 2) * 2)
            
            
        else:
            if temp_height % 2 or temp_width % 2:
                raise Exception("[Dvpp] width={} or height={} of output is odd"
                                .format(temp_width, temp_height))
            stride_width = temp_width
            stride_height = temp_height

        decode_out_buffer_size = int(int(stride_width *
                                         stride_height * 3) / 2)

        if need_malloc:
            temp_buffer, ret = acl.media.dvpp_malloc(decode_out_buffer_size)
            check_ret("acl.media.dvpp_malloc", ret)

        temp_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(temp_desc, temp_buffer)
        acl.media.dvpp_set_pic_desc_format(temp_desc, self._format)
        acl.media.dvpp_set_pic_desc_width(temp_desc, temp_width)
        acl.media.dvpp_set_pic_desc_height(temp_desc, temp_height)
        acl.media.dvpp_set_pic_desc_width_stride(temp_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(temp_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(temp_desc, decode_out_buffer_size)
        return temp_desc, temp_buffer, decode_out_buffer_size

    def forward(self,
                img_buffer,
                img_buffer_size,
                img_width,
                img_height):
        print('[Dvpp] vpc resize stage:')
        
        # because here the input is from vdec, the input stride is 16*2;
        self._resize_in_desc_, self._decode_out_buffer, _resize_in_size = \
            self.gen_tensor_desc(img_buffer,
                                 img_width,
                                 img_height,
                                 need_malloc=False)
        # the output of resize is vpc constraint, the output stride is 16*2 too.
        self._resize_out_desc, self._resize_out_dev, self._resize_out_size = \
            self.gen_tensor_desc(self._resize_out_dev,
                                 self._model_input_width,
                                 self._model_input_height,
                                 flag=False)

        if _resize_in_size != img_buffer_size:
            print("[Dvpp] self._resize_out_buffer_size:{} img_buffer_size:{}"
                  .format(_resize_in_size, img_buffer_size))
            raise Exception("[Dvpp] Size doesn't match")

        ret = acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                              self._resize_in_desc_,
                                              self._resize_out_desc,
                                              self._resize_config,
                                              self.stream)
        check_ret("acl.media.dvpp_vpc_resize_async", ret)

        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] vpc resize stage success')

    def run(self, img_buffer, img_buffer_size, img_width, img_height):
        self.forward(img_buffer,
                     img_buffer_size,
                     img_width,
                     img_height)
        return self._resize_out_dev, self._resize_out_size
