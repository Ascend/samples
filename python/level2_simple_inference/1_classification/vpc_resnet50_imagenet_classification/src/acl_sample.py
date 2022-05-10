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
import os
import argparse
import acl
from PIL import Image
from constant import ACL_MEMCPY_HOST_TO_DEVICE, IMG_EXT
from acl_model import Model, check_ret
from acl_dvpp import Dvpp
from acl_op import SingleOp


class Sample(object):
    """
    样例入口
    """
    def __init__(self,
                 device_id,
                 model_path,
                 model_input_width,
                 model_input_height):
        self.device_id = device_id      # int
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.context = None             # pointer

        self.input_data = None
        self.output_data = None
        self.model_desc = None          # pointer when using
        self.load_input_dataset = None
        self.load_output_dataset = None
        self.init_resource()

        self._model_input_width = model_input_width
        self._model_input_height = model_input_height

        self.model_process = Model(self.context,
                                   self.stream,
                                   self.model_path)

        self.dvpp_process = Dvpp(self.stream,
                                 model_input_width,
                                 model_input_height)

        self.sing_op = SingleOp(self.stream)

    def release_resource(self):
        if self.model_process:
            del self.model_process

        if self.dvpp_process:
            del self.dvpp_process

        if self.sing_op:
            del self.sing_op

        if self.stream:
            acl.rt.destroy_stream(self.stream)

        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("[Sample] class Samle release source success")

    def init_resource(self):
        print("[Sample] init resource stage:")
        acl.init()
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)
        print("[Sample] init resource stage success")

    def _transfer_to_device(self, img_path, dtype=np.uint8):
        img = np.fromfile(img_path, dtype=dtype)
        if "bytes_to_ptr" in dir(acl.util):
            bytes_data = img.tobytes()
            img_ptr = acl.util.bytes_to_ptr(bytes_data)
        else:
            img_ptr = acl.util.numpy_to_ptr(img)
        img_buffer_size = img.itemsize * img.size
        img_device, ret = acl.media.dvpp_malloc(img_buffer_size)
        check_ret("acl.media.dvpp_malloc", ret)
        ret = acl.rt.memcpy(img_device,
                            img_buffer_size,
                            img_ptr,
                            img_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        return img_device, img_buffer_size

    def forward(self, img_dict):
        img_path, _ = img_dict["path"], img_dict["dtype"]
        # copy images to device
        with Image.open(img_path) as image_file:
            width, height = image_file.size
            print("[Sample] width:{} height:{}".format(width, height))
            print("[Sample] image:{}".format(img_path))
        img_device, img_buffer_size = \
            self._transfer_to_device(img_path, img_dict["dtype"])

        # decode and resize
        dvpp_output_buffer, dvpp_output_size = \
            self.dvpp_process.run(img_device,
                                  img_buffer_size,
                                  width,
                                  height)
        self.model_process.run(
            dvpp_output_buffer,
            dvpp_output_size)
        self.sing_op.run(self.model_process.get_result())
        if img_device:
            acl.media.dvpp_free(img_device)


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--model_path', type=str,
                        default=os.path.join(current_dir, "../model/resnet50_aipp.om"))
    parser.add_argument('--model_input_width', type=int, default=224)
    parser.add_argument('--model_input_weight', type=int, default=224)
    parser.add_argument('--images_path', type=str, default=os.path.join(current_dir, "../data"))
    args = parser.parse_args()
    print("Using device id:{}\nmodel path:{}\nimages path:{}"
          .format(args.device, args.model_path, args.images_path))

    sample = Sample(args.device,
                    args.model_path,
                    args.model_input_width,
                    args.model_input_weight)
    images_list = [os.path.join(args.images_path, img)
                   for img in os.listdir(args.images_path)
                   if os.path.splitext(img)[1] in IMG_EXT]

    for image in images_list:
        img_dict_src = {"path": image, "dtype": np.uint8}
        sample.forward(img_dict_src)
    sample.release_resource()
