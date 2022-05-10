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
import acl
import numpy as np
from acl_vdec import Vdec
from acl_model import Model
from acl_util import check_ret
from acl_dvpp import Dvpp


class Sample(object):
    def __init__(self,
                 device_id,
                 model_path,
                 vdec_out_path,
                 model_input_width,
                 model_input_height):
        self.device_id = device_id  # int
        self.model_path = model_path  # string
        self.context = None  # pointer
        self.stream = None
        self.model_input_width = model_input_width
        self.model_input_height = model_input_height,
        self.init_resource()
        self.model_process = Model(self.context,
                                   self.stream,
                                   model_path)
        self.vdec_process = Vdec(self.context,
                                 self.stream,
                                 vdec_out_path)
        self.dvpp_process = Dvpp(self.stream,
                                 model_input_width,
                                 model_input_height)
        self.model_input_width = model_input_width
        self.model_input_height = model_input_height
        self.vdec_out_path = vdec_out_path

    def init_resource(self):
        print("init resource stage:")
        acl.init()
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)
        print("init resource stage success")

    def release_resource(self):
        print('[Sample] release source stage:')
        if self.dvpp_process:
            del self.dvpp_process

        if self.model_process:
            del self.model_process

        if self.vdec_process:
            del self.vdec_process

        if self.stream:
            ret = acl.rt.destroy_stream(self.stream)
            check_ret("acl.rt.destroy_stream", ret)

        if self.context:
            ret = acl.rt.destroy_context(self.context)
            check_ret("acl.rt.destroy_context", ret)

        ret = acl.rt.reset_device(self.device_id)
        check_ret("acl.rt.reset_device", ret)
        ret = acl.finalize()
        check_ret("acl.finalize", ret)
        print('[Sample] release source stage success')

    def _transfer_to_device(self, img):
        img_device = img["buffer"]
        img_buffer_size = img["size"]
        
        '''
        if the buffer is not in device, need to copy to device, but here, the data is from vdec, no need to copy.
        '''
        return img_device, img_buffer_size

    def forward(self, temp):
        _, input_width, input_height, _ = temp

        # vdec process，note：the input is h264 file，vdec output datasize need to be computed by strided width and height by 16*2
        self.vdec_process.run(temp)
        
        images_buffer = self.vdec_process.get_image_buffer()
        if images_buffer:
            for img_buffer in images_buffer:
                img_device, img_buffer_size = \
                    self._transfer_to_device(img_buffer)
                
                print("vdec output, img_buffer_size = ", img_buffer_size)
                # vpc process, parameters：vdec output buffer and size, original picture width and height.
                dvpp_output_buffer, dvpp_output_size = \
                    self.dvpp_process.run(img_device,
                                          img_buffer_size,
                                          input_width,
                                          input_height)
           
                ret = acl.media.dvpp_free(img_device)
                check_ret("acl.media.dvpp_free", ret)
                
                self.model_process.run(dvpp_output_buffer,
                                       dvpp_output_size)


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))
    MODEL_PATH = os.path.join(current_dir, "../model/resnet50_aipp.om")
    VEDIO_PATH = os.path.join(current_dir, "../data/vdec_h265_1frame_rabbit_1280x720.h265")
    VDEC_OUT_PATH = os.path.join(current_dir, "../vdec_out")

    if not os.path.exists(VDEC_OUT_PATH):
        os.makedirs(VDEC_OUT_PATH)

    sample = Sample(0, MODEL_PATH, VDEC_OUT_PATH, 224, 224)
    vedio_list = [VEDIO_PATH,
                  1280,  # width
                  720,  # height
                  np.uint8]  # dtype
    # vedio_list = ["./data/test_29.h264",
    #               720,  # width
    #               1280,  # height
    #               np.uint8]  # dtype
    
    
    sample.forward(vedio_list)
    sample.release_resource()
