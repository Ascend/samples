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
from constant import ImageType, MemcpyType


PIXEL_FORMAT_YUV_SEMIPLANAR_420 = \
    ImageType.PIXEL_FORMAT_YUV_SEMIPLANAR_420.value
ACL_MEMCPY_DEVICE_TO_HOST = MemcpyType.ACL_MEMCPY_DEVICE_TO_HOST.value
ACL_MEMCPY_HOST_TO_DEVICE = MemcpyType.ACL_MEMCPY_HOST_TO_DEVICE.value


def check_ret(message, ret):
    if ret != 0:
        raise Exception("{} failed ret={}"
                        .format(message, ret))

def numpy_2_ptr(np_data):
    if "bytes_to_ptr" in dir(acl.util):
        bytes_data = np_data.tobytes()
        ptr = acl.util.bytes_to_ptr(bytes_data)
        return ptr, bytes_data
    else:
        ptr = acl.util.numpy_to_ptr(np_data)
        return ptr, None


class Dvpp(object):
    def __init__(self, stream, mdl_w, mdl_h):
        self.mdl_w = mdl_w
        self.mdl_h = mdl_h

        self._dvpp_channel_desc = 0

        self._resize_config = 0
        self._jpege_config = 0

        self._roi_dic = {}
        self._dev_dic = {}
        self._dev_size_dic = {}
        self._pic_desc_dic = {}
        self._batch_pic_desc = {}
        self._batch_crop_roi_list = []
        self._batch_crop_paste_roi_list = []

        self._format = PIXEL_FORMAT_YUV_SEMIPLANAR_420
        self.stream = stream
        self.init_resource()

    def __del__(self):
        print("[Dvpp] class Dvpp release source start:")
        if self._resize_config:
            ret = acl.media.dvpp_destroy_resize_config(self._resize_config)
            check_ret("acl.media.dvpp_destroy_resize_config", ret)

        if self._jpege_config:
            ret = acl.media.dvpp_destroy_jpege_config(self._jpege_config)
            check_ret("acl.media.dvpp_destroy_jpege_config", ret)

        if self._dvpp_channel_desc:
            ret = acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            check_ret("acl.media.dvpp_destroy_channel", ret)
            ret = acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)
            check_ret("acl.media.dvpp_destroy_channel_desc", ret)

        for key in self._pic_desc_dic.keys():
            if self._pic_desc_dic[key]:
                ret = acl.media.dvpp_destroy_pic_desc(self._pic_desc_dic[key])
                check_ret("acl.media.dvpp_destroy_pic_desc", ret)

        for key in self._batch_pic_desc.keys():
            if self._batch_pic_desc[key]:
                ret = acl.media.dvpp_destroy_batch_pic_desc(
                    self._batch_pic_desc[key])
                check_ret("acl.media.dvpp_destroy_batch_pic_desc", ret)

        self.destroy()
        print("[Dvpp] class Dvpp release source success")

    def destroy(self):
        for key in self._roi_dic.keys():
            if self._roi_dic[key]:
                ret = acl.media.dvpp_destroy_roi_config(self._roi_dic[key])
                check_ret("acl.media.dvpp_destroy_roi_config", ret)

        for key in self._dev_dic.keys():
            if self._dev_dic[key]:
                ret = acl.media.dvpp_free(self._dev_dic[key])
                check_ret("acl.media.dvpp_free", ret)

        for item in self._batch_crop_roi_list:
            if item:
                ret = acl.media.dvpp_destroy_roi_config(item)
                check_ret("acl.media.dvpp_destroy_roi_config", ret)

        for item in self._batch_crop_paste_roi_list:
            if item:
                ret = acl.media.dvpp_destroy_roi_config(item)
                check_ret("acl.media.dvpp_destroy_roi_config", ret)

    def init_resource(self):
        print("[Dvpp] class Dvpp init resource start:")
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        check_ret("acl.media.dvpp_create_channel", ret)
        print("[Dvpp] class Dvpp init resource success")

    """
    jpeg process
    """
    def gen_jpeg_pic_desc(self, opt, width_src, height_src, buffer_size,
                          dev_buf, w_align=16, h_align=2):
        key = opt + '_in'
        self._dev_dic[key] = dev_buf
        self._dev_size_dic[key] = buffer_size

        align_width = ((width_src + w_align - 1) // w_align) * w_align
        align_height = ((height_src + h_align - 1) // h_align) * h_align

        pic_desc = acl.media.dvpp_create_pic_desc()

        # for jpeg
        if opt == "decode":
            out_buffer_size = (align_width * align_height * 3) // 2
            print("[Dvpp] pic desc = {},{},{},{}".format(
                align_width, align_height,
                out_buffer_size, buffer_size))
            out_dev, ret = acl.media.dvpp_malloc(out_buffer_size)
            check_ret("acl.media.dvpp_malloc", ret)
            key = opt + '_out'
            self._dev_dic[key] = out_dev
            self._dev_size_dic[key] = out_buffer_size

            acl.media.dvpp_set_pic_desc_data(pic_desc, out_dev)
            acl.media.dvpp_set_pic_desc_size(pic_desc, out_buffer_size)
        # encode set input
        else:
            acl.media.dvpp_set_pic_desc_data(pic_desc, dev_buf)
            acl.media.dvpp_set_pic_desc_size(pic_desc, buffer_size)
        acl.media.dvpp_set_pic_desc_format(pic_desc, self._format)
        acl.media.dvpp_set_pic_desc_width(pic_desc, width_src)
        acl.media.dvpp_set_pic_desc_height(pic_desc, height_src)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, align_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, align_height)

        self._pic_desc_dic[opt] = pic_desc
        return pic_desc

    def jpeg_encode_config(self, level):
        # set jpeg config for output size predicting use
        self._jpege_config = acl.media.dvpp_create_jpege_config()
        ret = acl.media.dvpp_set_jpege_config_level(self._jpege_config, level)
        check_ret("dvpp_set_jpege_config_level", ret)
        out_buffer_size, ret = acl.media.dvpp_jpeg_predict_enc_size(
            self._pic_desc_dic['encode'], self._jpege_config)
        check_ret("dvpp_jpeg_predict_enc_size", ret)
        out_dev, ret = acl.media.dvpp_malloc(out_buffer_size)

        check_ret("dvpp_malloc", ret)
        self._dev_dic['encode_out'] = out_dev
        self._dev_size_dic['encode_out'] = out_buffer_size

    def process_jpeg_enc(self, path, width_src, height_src):
        print('[Dvpp] jpeg encode process start:')
        np_yuv = np.fromfile(path, dtype=np.byte)
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)

        w_align, h_align = 16, 2
        align_width = ((width_src + w_align - 1) // w_align) * w_align
        align_height = ((height_src + h_align - 1) // h_align) * h_align
        buffer_size = (align_width * align_height * 3) // 2
        in_buf_dev, ret = acl.media.dvpp_malloc(buffer_size)
        check_ret("acl.media.dvpp_malloc", ret)
        ret = acl.rt.memcpy(in_buf_dev, buffer_size, np_yuv_ptr, buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        print('[Dvpp] jpeg encode set picture description')
        pic_desc = self.gen_jpeg_pic_desc("encode", width_src, height_src,
                                          buffer_size, in_buf_dev,
                                          w_align, h_align)
        print('[Dvpp] jpeg encode config')
        self.jpeg_encode_config(100)
        np_out_size = np.array([self._dev_size_dic['encode_out']],
                               dtype=np.int32)
        np_out_size_ptr, bytes_data = numpy_2_ptr(np_out_size)
        ret = acl.media.dvpp_jpeg_encode_async(self._dvpp_channel_desc,
                                               pic_desc,
                                               self._dev_dic['encode_out'],
                                               np_out_size_ptr,
                                               self._jpege_config,
                                               self.stream)
        check_ret("acl.media.dvpp_jpeg_encode_async", ret)

        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] jpeg encode success')
        size = int(np_out_size[0])
        np_output = np.zeros(size, dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr, size, self._dev_dic['encode_out'],
                            size, ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output

    def process_jpeg_dec(self, path):
        print('[Dvpp] jpeg decode process start:')
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_jpg_ptr, bytes_data = numpy_2_ptr(np_yuv)
        in_dev, ret = acl.media.dvpp_malloc(in_buffer_size)
        check_ret("acl.media.dvpp_malloc", ret)
        ret = acl.rt.memcpy(in_dev, in_buffer_size, np_jpg_ptr,
                            in_buffer_size, ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        width_src, height_src, channel, ret = \
            acl.media.dvpp_jpeg_get_image_info(np_jpg_ptr, in_buffer_size)
        print("[Dvpp] path = {}, width_src = {}, height_src = {}\n".format(
            path, width_src, height_src))

        w_align, h_align = 128, 16

        print('[Dvpp] jpeg decode set picture description')
        pic_desc = self.gen_jpeg_pic_desc("decode", width_src, height_src,
                                          in_buffer_size, in_dev,
                                          w_align, h_align)
        ret = acl.media.dvpp_jpeg_decode_async(self._dvpp_channel_desc,
                                               in_dev, in_buffer_size,
                                               pic_desc,
                                               self.stream)
        check_ret("acl.media.dvpp_jpeg_decode_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] jpeg decode success')

        size = self._dev_size_dic['decode_out']
        np_output = np.zeros(size, dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr, size, self._dev_dic['decode_out'],
                            size, ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output, acl.media.dvpp_get_pic_desc_width(pic_desc), \
            acl.media.dvpp_get_pic_desc_height(pic_desc)

    def destroy_jpeg_resource(self):
        print("[Dvpp] jpeg release source start:")
        for key in self._pic_desc_dic.keys():
            if self._pic_desc_dic[key]:
                ret = acl.media.dvpp_destroy_pic_desc(self._pic_desc_dic[key])
                check_ret("acl.media.dvpp_destroy_pic_desc", ret)
                self._pic_desc_dic[key] = 0

        if self._jpege_config:
            ret = acl.media.dvpp_destroy_jpege_config(self._jpege_config)
            check_ret("acl.media.dvpp_destroy_jpege_config", ret)
            self._jpege_config = 0

        for key in self._dev_dic.keys():
            if self._dev_dic[key]:
                ret = acl.media.dvpp_free(self._dev_dic[key])
                check_ret("acl.media.dvpp_free", ret)
                self._dev_dic[key] = 0

        print("[Dvpp] class jpeg release source success")

    """
    vpc process
    """
    def gen_vpc_8k_pic_desc(self, opt, width_src, height_src):
        align_width = width_src
        align_height = height_src

        buf_size = (align_width * align_height * 3) // 2

        pic_desc = acl.media.dvpp_create_pic_desc()
        self._pic_desc_dic[opt] = pic_desc

        # for vpc
        dev, ret = acl.media.dvpp_malloc(buf_size)
        check_ret("acl.media.dvpp_malloc", ret)
        self._dev_dic[opt] = dev
        self._dev_size_dic[opt] = buf_size
        acl.media.dvpp_set_pic_desc_data(pic_desc, dev)
        acl.media.dvpp_set_pic_desc_size(pic_desc, buf_size)

        acl.media.dvpp_set_pic_desc_format(pic_desc, self._format)
        acl.media.dvpp_set_pic_desc_width(pic_desc, align_width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, align_height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, align_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, align_height)

    def gen_vpc_pic_desc(self, width_src, height_src, opt, w_align=16,
                         h_align=2, pic_desc=None):
        align_width = ((width_src + w_align - 1) // w_align) * w_align
        align_height = ((height_src + h_align - 1) // h_align) * h_align

        buf_size = (align_width * align_height * 3) // 2

        if pic_desc is None:
            pic_desc = acl.media.dvpp_create_pic_desc()
            self._pic_desc_dic[opt] = pic_desc

        # for vpc
        dev, ret = acl.media.dvpp_malloc(buf_size)
        check_ret("acl.media.dvpp_malloc", ret)
        self._dev_dic[opt] = dev
        self._dev_size_dic[opt] = buf_size
        acl.media.dvpp_set_pic_desc_data(pic_desc, dev)
        acl.media.dvpp_set_pic_desc_size(pic_desc, buf_size)

        acl.media.dvpp_set_pic_desc_format(pic_desc, self._format)
        acl.media.dvpp_set_pic_desc_width(pic_desc, width_src)
        acl.media.dvpp_set_pic_desc_height(pic_desc, height_src)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, align_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, align_height)

    def process_vpc_resize(self, path, width_src, height_src):
        print("[Dvpp] vpc resize process start:")
        w_align, h_align = 16, 2

        # create input picture description
        print("[Dvpp] vpc resize set input pic desc")
        self.gen_vpc_pic_desc(width_src, height_src, "in_resize",
                              w_align, h_align)

        # create output picture description
        print("[Dvpp] vpc resize set output pic desc")
        self.gen_vpc_pic_desc(self.mdl_w, self.mdl_h, "out_resize", w_align,
                              h_align)

        # copy host to device
        print("[Dvpp] vpc resize copy input to device")
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)
        ret = acl.rt.memcpy(self._dev_dic["in_resize"], in_buffer_size,
                            np_yuv_ptr, in_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        # resize
        self._resize_config = acl.media.dvpp_create_resize_config()
        ret = \
            acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                            self._pic_desc_dic['in_resize'],
                                            self._pic_desc_dic['out_resize'],
                                            self._resize_config,
                                            self.stream)
        check_ret("acl.media.dvpp_vpc_resize_async", ret)
        print("[Dvpp] vpc resize process success")

        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)

        # copy data from device to host
        np_output = np.zeros(self._dev_size_dic.get('out_resize'), dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr,
                            self._dev_size_dic.get('out_resize'),
                            self._dev_dic.get("out_resize"),
                            self._dev_size_dic.get('out_resize'),
                            ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output, self._dev_dic["out_resize"], \
            self._dev_size_dic['out_resize']

    def process_vpc_8k_resize(self, path, width_src, height_src):
        print("[Dvpp] vpc 8k resize process start:")
        # create input picture description
        print("[Dvpp] vpc 8k resize set input pic desc")
        self.gen_vpc_8k_pic_desc('8k_in_resize', 8192, 8192)

        # create output picture description
        print("[Dvpp] vpc 8k resize set output pic desc")
        self.gen_vpc_8k_pic_desc('8k_out_resize', 4000, 4000)

        # copy host to device
        print("[Dvpp] vpc 8k resize copy input to device")
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)
        ret = acl.rt.memcpy(self._dev_dic["8k_in_resize"], in_buffer_size,
                            np_yuv_ptr, in_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        # resize
        self._resize_config = acl.media.dvpp_create_resize_config()
        ret = acl.media.dvpp_vpc_resize_async(
                                        self._dvpp_channel_desc,
                                        self._pic_desc_dic['8k_in_resize'],
                                        self._pic_desc_dic['8k_out_resize'],
                                        self._resize_config,
                                        self.stream)
        check_ret("acl.media.dvpp_vpc_resize_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[Dvpp] vpc 8k resize process success")

        # copy data from device to host
        np_output = np.zeros(self._dev_size_dic.get('8k_out_resize'),
                             dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr,
                            self._dev_size_dic.get('8k_out_resize'),
                            self._dev_dic.get("8k_out_resize"),
                            self._dev_size_dic.get('8k_out_resize'),
                            ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output, self._dev_dic.get("8k_out_resize"), \
            self._dev_size_dic.get('8k_out_resize')

    def destroy_resize_resource(self):
        print("[Dvpp] resize release source start:")
        for key in self._pic_desc_dic.keys():
            if self._pic_desc_dic[key]:
                ret = acl.media.dvpp_destroy_pic_desc(self._pic_desc_dic[key])
                check_ret("acl.media.dvpp_destroy_pic_desc", ret)
                self._pic_desc_dic[key] = 0

        if self._resize_config:
            ret = acl.media.dvpp_destroy_resize_config(self._resize_config)
            check_ret("acl.media.dvpp_destroy_resize_config", ret)
            self._resize_config = 0

        for key in self._dev_dic.keys():
            if self._dev_dic[key]:
                ret = acl.media.dvpp_free(self._dev_dic[key])
                check_ret("acl.media.dvpp_free", ret)
                self._dev_dic[key] = 0

        print("[Dvpp] resize release source success")

    def process_vpc_crop(self, path, width_src, height_src):
        print("[Dvpp] vpc crop process start:")
        w_align, h_align = 16, 2

        # create input picture description
        self.gen_vpc_pic_desc(width_src, height_src, "in_crop", w_align,
                              h_align)

        # create output picture description
        self.gen_vpc_pic_desc(self.mdl_w, self.mdl_h, "out_crop", w_align,
                              h_align)

        # copy from host to device
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)
        ret = acl.rt.memcpy(self._dev_dic["in_crop"], in_buffer_size,
                            np_yuv_ptr, in_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)

        self._roi_dic['crop'] = acl.media.dvpp_create_roi_config(
            self.mdl_w, width_src - 1,
            self.mdl_h, height_src - 1)
        ret = acl.media.dvpp_vpc_crop_async(
            self._dvpp_channel_desc,
            self._pic_desc_dic['in_crop'],
            self._pic_desc_dic['out_crop'],
            self._roi_dic['crop'],
            self.stream)
        check_ret("acl.media.dvpp_vpc_crop_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[Dvpp] vpc crop process success")

        np_output = np.zeros(self._dev_size_dic['out_crop'], dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr,
                            self._dev_size_dic.get('out_crop'),
                            self._dev_dic.get("out_crop"),
                            self._dev_size_dic.get('out_crop'),
                            ACL_MEMCPY_DEVICE_TO_HOST)

        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output, self._dev_dic["out_crop"], \
            self._dev_size_dic['out_crop']

    def process_vpc_crop_paste(self, path, width_src, height_src):
        print("[Dvpp] vpc crop and paste process start:")
        w_align, h_align = 16, 2

        # create input picture description
        self.gen_vpc_pic_desc(width_src, height_src, "in_crop_paste",
                              w_align, h_align)

        # create output picture description
        self.gen_vpc_pic_desc(width_src // 2, height_src // 2,
                              "out_crop_paste", w_align, h_align)

        # copy from host to device
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)
        ret = acl.rt.memcpy(self._dev_dic["in_crop_paste"], in_buffer_size,
                            np_yuv_ptr, in_buffer_size,
                            ACL_MEMCPY_HOST_TO_DEVICE)
        check_ret("acl.rt.memcpy", ret)
        # from input
        self._roi_dic['crop'] = \
            acl.media.dvpp_create_roi_config(self.mdl_w,
                                             width_src - 1, self.mdl_h,
                                             height_src - 1)
        # from output
        self._roi_dic['paste'] = acl.media.dvpp_create_roi_config(32, 215,
                                                                  64, 215)
        ret = acl.media.dvpp_vpc_crop_and_paste_async(
                self._dvpp_channel_desc,
                self._pic_desc_dic.get('in_crop_paste'),
                self._pic_desc_dic.get('out_crop_paste'),
                self._roi_dic.get('crop'),
                self._roi_dic.get('paste'),
                self.stream)
        check_ret("acl.media.dvpp_vpc_crop_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[Dvpp] vpc crop process success")

        np_output = np.zeros(self._dev_size_dic['out_crop_paste'],
                             dtype=np.byte)
        np_output_ptr, bytes_data = numpy_2_ptr(np_output)
        ret = acl.rt.memcpy(np_output_ptr,
                            self._dev_size_dic.get('out_crop_paste'),
                            self._dev_dic.get("out_crop_paste"),
                            self._dev_size_dic.get('out_crop_paste'),
                            ACL_MEMCPY_DEVICE_TO_HOST)

        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_output = np.frombuffer(bytes_data, dtype=np.byte)
        return np_output, self._dev_dic["out_crop_paste"], \
            self._dev_size_dic['out_crop_paste']

    def get_pic_desc_data(self, pic_desc):
        pic_data = acl.media.dvpp_get_pic_desc_data(pic_desc)
        pic_data_size = acl.media.dvpp_get_pic_desc_size(pic_desc)
        ret_code = acl.media.dvpp_get_pic_desc_ret_code(pic_desc)
        check_ret("acl.media.dvpp_get_pic_desc_ret_code", ret_code)

        # D2H
        np_pic = np.zeros(pic_data_size, dtype=np.byte)
        np_pic_ptr, bytes_data = numpy_2_ptr(np_pic)
        ret = acl.rt.memcpy(np_pic_ptr, pic_data_size,
                            pic_data, pic_data_size,
                            ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)
        if "bytes_to_ptr" in dir(acl.util):
            np_pic = np.frombuffer(bytes_data, dtype=np.byte)
        return np_pic

    def load_data_from_file(self, path, in_batch_size,
                            out_batch_size, width_src, height_src):
        self._batch_pic_desc['out_batch'] = \
            acl.media.dvpp_create_batch_pic_desc(out_batch_size)
        self._batch_pic_desc['in_batch'] = \
            acl.media.dvpp_create_batch_pic_desc(in_batch_size)

        # load data from file
        np_yuv = np.fromfile(path, dtype=np.byte)
        in_buffer_size = np_yuv.itemsize * np_yuv.size
        np_yuv_ptr, bytes_data = numpy_2_ptr(np_yuv)

        roi_list = []
        w_align, h_align = 16, 2
        for i in range(in_batch_size):
            opt = 'in_batch_crop_' + str(i)
            input_desc = acl.media.dvpp_get_pic_desc(
                self._batch_pic_desc['in_batch'], i)
            self.gen_vpc_pic_desc(width_src, height_src, opt,
                                  w_align, h_align, input_desc)

            # copy from host to device
            ret = acl.rt.memcpy(self._dev_dic[opt], in_buffer_size,
                                np_yuv_ptr, in_buffer_size,
                                ACL_MEMCPY_HOST_TO_DEVICE)
            check_ret("acl.rt.memcpy", ret)
            roi_list.append(out_batch_size // in_batch_size)

        return roi_list, w_align, h_align

    def process_vpc_batchcrop_asyn(self, path, in_batch_size,
                                   out_batch_size, width_src, height_src):
        print("[Dvpp] batch vpc crop process start:")
        roi_list, w_align, h_align = self.load_data_from_file(
            path, in_batch_size, out_batch_size, width_src, height_src)

        for i in range(out_batch_size):
            str_opt = 'out_batch_crop_' + str(i)
            output_desc = acl.media.dvpp_get_pic_desc(
                self._batch_pic_desc['out_batch'], i)
            self.gen_vpc_pic_desc(self.mdl_w, self.mdl_h,
                                  str_opt, w_align, h_align, output_desc)
            if i == 0:
                crop_area = acl.media.dvpp_create_roi_config(
                    width_src - self.mdl_w, width_src - 1,
                    height_src - self.mdl_h, height_src - 1)
            else:
                crop_area = acl.media.dvpp_create_roi_config(
                    0, width_src - self.mdl_w - 1,
                    height_src - self.mdl_h, height_src - 1)
            self._batch_crop_roi_list.append(crop_area)

        total_num = 0
        for i in range(in_batch_size):
            total_num += roi_list[i]
        if out_batch_size % in_batch_size != 0:
            roi_list[-1] = out_batch_size - total_num + roi_list[-1]
        _, ret = acl.media.dvpp_vpc_batch_crop_async(
            self._dvpp_channel_desc,
            self._batch_pic_desc.get('in_batch'), roi_list,
            self._batch_pic_desc.get('out_batch'),
            self._batch_crop_roi_list, self.stream)
        check_ret("acl.media.dvpp_vpc_batch_crop_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[Dvpp] batch vpc crop process success")
        np_list = []
        for i in range(out_batch_size):
            output_desc = acl.media.dvpp_get_pic_desc(
                self._batch_pic_desc['out_batch'], i)
            np_output = self.get_pic_desc_data(output_desc)
            np_list.append(np_output)

        return np_list

    def process_vpc_batchcrop_paste_asyn(self, path, in_batch_size,
                                         out_batch_size, width_src,
                                         height_src):
        print("[Dvpp] batch vpc crop and paste process start:")
        roi_list, w_align, h_align = self.load_data_from_file(
            path, in_batch_size, out_batch_size, width_src, height_src)

        for i in range(out_batch_size):
            opt = 'out_batch_crop_' + str(i)
            output_desc = acl.media.dvpp_get_pic_desc(
                self._batch_pic_desc['out_batch'], i)
            self.gen_vpc_pic_desc(self.mdl_w, self.mdl_h,
                                  opt, w_align, h_align, output_desc)
            if i == 0:
                crop_area = acl.media.dvpp_create_roi_config(
                    width_src - self.mdl_w, width_src - 1,
                    height_src - self.mdl_h, height_src - 1)
            else:
                crop_area = acl.media.dvpp_create_roi_config(
                    0, width_src - self.mdl_w - 1, height_src - self.mdl_h,
                    height_src - 1)
            paste_area = acl.media.dvpp_create_roi_config(
                self.mdl_w // 2, self.mdl_w - 1,
                self.mdl_h // 2, self.mdl_h - 1)
            self._batch_crop_roi_list.append(crop_area)
            self._batch_crop_paste_roi_list.append(paste_area)

        total_num = 0
        for i in range(in_batch_size):
            total_num += roi_list[i]
        if out_batch_size % in_batch_size != 0:
            roi_list[-1] = out_batch_size - total_num + roi_list[-1]
        _, ret = acl.media.dvpp_vpc_batch_crop_and_paste_async(
            self._dvpp_channel_desc, self._batch_pic_desc.get('in_batch'),
            roi_list, self._batch_pic_desc.get('out_batch'),
            self._batch_crop_roi_list, self._batch_crop_paste_roi_list,
            self.stream)
        check_ret("acl.media.dvpp_vpc_batch_crop_and_paste_async", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[Dvpp] vpc batch crop and paste process success")
        np_list = []
        for i in range(out_batch_size):
            output_desc = acl.media.dvpp_get_pic_desc(
                self._batch_pic_desc.get('out_batch'), i)
            np_output = self.get_pic_desc_data(output_desc)
            np_list.append(np_output)

        return np_list

    def destroy_crop_resource(self):
        print("[Dvpp] crop release source start:")
        for key in self._pic_desc_dic.keys():
            if self._pic_desc_dic[key]:
                ret = acl.media.dvpp_destroy_pic_desc(self._pic_desc_dic[key])
                check_ret("acl.media.dvpp_destroy_pic_desc", ret)
                self._pic_desc_dic[key] = 0

        for key in self._batch_pic_desc.keys():
            if self._batch_pic_desc[key]:
                ret = acl.media.dvpp_destroy_batch_pic_desc(
                    self._batch_pic_desc[key])
                check_ret("acl.media.dvpp_destroy_batch_pic_desc", ret)
                self._batch_pic_desc[key] = 0

        for key in self._roi_dic.keys():
            if self._roi_dic[key]:
                ret = acl.media.dvpp_destroy_roi_config(self._roi_dic[key])
                check_ret("acl.media.dvpp_destroy_roi_config", ret)
                self._roi_dic[key] = 0

        for key in self._dev_dic.keys():
            if self._dev_dic[key]:
                ret = acl.media.dvpp_free(self._dev_dic[key])
                check_ret("acl.media.dvpp_free", ret)
                self._dev_dic[key] = 0

        for i in range(len(self._batch_crop_roi_list)):
            if self._batch_crop_roi_list[i]:
                ret = acl.media.dvpp_destroy_roi_config(
                    self._batch_crop_roi_list[i])
                check_ret("acl.media.dvpp_destroy_roi_config", ret)
                self._batch_crop_roi_list[i] = 0

        for i in range(len(self._batch_crop_paste_roi_list)):
            if self._batch_crop_paste_roi_list[i]:
                ret = acl.media.dvpp_destroy_roi_config(
                    self._batch_crop_paste_roi_list[i])
                check_ret("acl.media.dvpp_destroy_roi_config", ret)
                self._batch_crop_paste_roi_list[i] = 0

        print("[Dvpp] crop release source success")
