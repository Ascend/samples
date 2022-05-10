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
import argparse
from PIL import Image
import acl
from constant import VPC_RESIZE, JPEG_ENC, VPC_CROP, \
    VPC_CROP_PASTE, VPC_8K_RESIZE, VPC_BATCH_CROP, VPC_BACTH_CROP_PASTE
from acl_model_process import Model
from acl_dvpp_process import Dvpp


def check_ret(message, ret):
    """
    功能简介：检测pyACL函数返回值是否正常，如果非0则会抛出异常
    参数：ret，pyACL函数返回值
    返回值：无
    """
    if ret != 0:
        raise Exception("{} failed ret={}"
                        .format(message, ret))


class Sample(object):
    def __init__(self, device_id, model_path, mdl_w, mdl_h):
        self.device_id = device_id
        self.model_path = model_path
        self.context = None
        self.stream = None

        self.init_resource()
        self.model_process = Model(self.context, self.stream, model_path)
        self.dvpp_process = Dvpp(self.stream, mdl_w, mdl_h)

    def release_resource(self):
        if self.model_process:
            del self.model_process

        if self.dvpp_process:
            del self.dvpp_process

        if self.stream:
            acl.rt.destroy_stream(self.stream)

        if self.context:
            acl.rt.destroy_context(self.context)

        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("[Sample] class Sample release source success")

    def init_resource(self):
        print("[Sample] init resource start:")
        ret = acl.init()
        check_ret("acl.init", ret)
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)
        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)
        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)
        print("[Sample] init resource success")

    def _get_image_w_h(self, img_path, img_type):
        width, height = 0, 0
        if img_type == 'yuv':
            h_w_str = img_path.split('/')[-1]
            width = int(h_w_str.split('_')[2])
            height = int(h_w_str.split('_')[3])
        elif img_type == 'jpg':
            with Image.open(img_path) as image_file:
                width, height = image_file.size

        return width, height

    def forward(self, img_dict):
        img_path, dvpp_type = img_dict["path"], img_dict["dvpp_type"]
        res_path = img_dict['result_path']
        img_type = img_dict['image_type']
        in_batch_size = img_dict['in_batch_size']
        out_batch_size = img_dict['out_batch_size']
        print("[Sample] image:{} res_path:{}".format(img_path, res_path))

        # get image size
        width, height = self._get_image_w_h(img_path, img_type)
        print("[Sample] width:{} height:{}".format(width, height))

        # pre-process
        tmp_file = img_path
        if dvpp_type == VPC_8K_RESIZE:
            pass
        elif dvpp_type == JPEG_ENC and img_type == 'yuv':
            out_np = self.dvpp_process.process_jpeg_enc(img_path, width,
                                                        height)
            tmp_file = res_path + '/' + \
                img_path.split('/')[-1].split('.')[0] + '.jpg'
            out_np.tofile(tmp_file)
            self.dvpp_process.destroy_jpeg_resource()
            return
        elif dvpp_type != JPEG_ENC and img_type == 'jpg':
            # decode
            out_np, width, height = self.dvpp_process.process_jpeg_dec(img_path)
            self.dvpp_process.destroy_jpeg_resource()
            tmp_file = res_path + '/' + \
                img_path.split('/')[-1].split('.')[0] + '.yuv'
            out_np.tofile(tmp_file)
        elif dvpp_type != JPEG_ENC and img_type != 'jpg':
            tmp_file = img_path
        else:
            return

        self.vpc_process(img_path, dvpp_type, res_path, tmp_file,
                         in_batch_size, out_batch_size, width, height)

        # vpc process
    def vpc_process(self, img_path, dvpp_type, res_path, tmp_file,
                    in_batch_size, out_batch_size, width, height):
        if dvpp_type == VPC_RESIZE:
            out_np, out_dev, out_size = self.dvpp_process.process_vpc_resize(
                tmp_file, width, height)
        elif dvpp_type == VPC_8K_RESIZE:
            out_np, _, _ = self.dvpp_process.process_vpc_8k_resize(
                tmp_file, width, height)
            out_file = res_path + '/' + \
                img_path.split('/')[-1].split('.')[0] + '_' + \
                str(dvpp_type) + '_' + '.yuv'
            out_np.tofile(out_file)
            self.dvpp_process.destroy_resize_resource()
            return
        elif dvpp_type == VPC_CROP:
            out_np, out_dev, out_size = self.dvpp_process.process_vpc_crop(
                tmp_file, width, height)
        elif dvpp_type == VPC_CROP_PASTE:
            out_np, out_dev, out_size = \
                self.dvpp_process.process_vpc_crop_paste(
                    tmp_file, width, height)
        elif dvpp_type == VPC_BATCH_CROP:
            print("[Sample] in_batch_size:{} in_batch_size:{}".format(
                in_batch_size, out_batch_size))
            out_np = self.dvpp_process.process_vpc_batchcrop_asyn(
                tmp_file, in_batch_size, out_batch_size, width, height)
            for i, item in enumerate(out_np):
                out_file = res_path + '/' + \
                    tmp_file.split('/')[-1].split('.')[0] + '_' + \
                    str(dvpp_type) + 'crop_{}_'.format(i) + '.yuv'
                item.tofile(out_file)
                print("[Sample]write out to file {} success".format(out_file))
            return
        elif dvpp_type == VPC_BACTH_CROP_PASTE:
            print("[Sample] in_batch_size:{} in_batch_size:{}".format(
                in_batch_size, out_batch_size))
            out_np = self.dvpp_process.process_vpc_batchcrop_paste_asyn(
                tmp_file, in_batch_size, out_batch_size, width, height)
            for i, item in enumerate(out_np):
                out_file = res_path + '/' + \
                    tmp_file.split('/')[-1].split('.')[0] + '_' + \
                    str(dvpp_type) + 'crop_{}_'.format(i) + '.yuv'
                item.tofile(out_file)
                print("[Sample]write out to file {} success".format(out_file))
            return
        else:
            return

        out_file = res_path + '/' + tmp_file.split('/')[-1].split('.')[0] + \
            '_' + str(dvpp_type) + '.yuv'
        out_np.tofile(out_file)

        # model process
        self.model_process.run(out_dev, out_size)
        self.dvpp_process.destroy_crop_resource()


if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--model_path', type=str,
                        default=os.path.join(current_dir, "../model/resnet50_aipp.om"))
    parser.add_argument('--model_input_width', type=int, default=224)
    parser.add_argument('--model_input_weight', type=int, default=224)
    parser.add_argument('--images_path', type=str,
                        default=os.path.join(current_dir, "../data/persian_cat_1024_1536_283.jpg"))
    parser.add_argument('--dvpp_type', type=int, default=0)
    parser.add_argument('--result_path', type=str,
                        default=os.path.join(current_dir, "../vpc_out"))
    parser.add_argument('--image_type', type=str, default="jpg")
    parser.add_argument('--in_batch_size', type=int, default=1)
    parser.add_argument('--out_batch_size', type=int, default=8)
    args = parser.parse_args()
    print("Using device id:{}\nmodel path:{}\nimages path:{}\n"
          "result path:{}\ndvpp type:{}\n"
          .format(args.device, args.model_path, args.images_path,
                  args.result_path, args.dvpp_type))

    if not os.path.exists(args.result_path):
        os.mkdir(args.result_path)

    sample = Sample(args.device, args.model_path,
                    args.model_input_width,
                    args.model_input_weight)

    img_dict_data = {
        "path": args.images_path,
        "dvpp_type": args.dvpp_type,
        'result_path': args.result_path,
        'image_type': args.image_type,
        'in_batch_size': args.in_batch_size,
        'out_batch_size': args.out_batch_size
    }

    sample.forward(img_dict_data)
    sample.release_resource()
