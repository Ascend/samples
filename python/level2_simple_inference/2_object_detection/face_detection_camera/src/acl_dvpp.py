import numpy as np

import acl
from utils import *
from atlas_utils.acl_image import AclImage

class Dvpp():
    def __init__(self, acl_resource):
        self._stream = acl_resource.stream
        self._run_mode = acl_resource.run_mode
        self._dvpp_channel_desc = None
        ret = self._init_resource()
        if ret == FAILED:
            raise Exception("Dvpp init failed")

    def __del__(self):
        if self._resize_config:
            acl.media.dvpp_destroy_resize_config(self._resize_config)

        if self._dvpp_channel_desc:
            acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)

        if self._jpege_config:
            acl.media.dvpp_destroy_jpege_config(self._jpege_config)

    def _init_resource(self):
        #创建dvpp channel
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        if ret != ACL_ERROR_NONE:
            print("Dvpp create channel failed")
            return FAILED
        #创建resize配置
        self._resize_config = acl.media.dvpp_create_resize_config()
        #创建yuv转jpeg的配置
        self._jpege_config = acl.media.dvpp_create_jpege_config()
        ret = acl.media.dvpp_set_jpege_config_level (self._jpege_config, 100)
        if ret != ACL_ERROR_NONE:
            print("Dvpp set jpege config failed")
            return FAILED

        return SUCCESS

    def _gen_input_pic_desc(self, image,
                            width_align_factor=16, height_align_factor=2):
        #创建图片输入desc
        stride_width = align_up(image.width, width_align_factor)
        stride_height = align_up(image.height, height_align_factor)

        pic_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(pic_desc, image.data())
        acl.media.dvpp_set_pic_desc_format(pic_desc,
                                           PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(pic_desc, image.width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, image.height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(pic_desc, image.size)

        return pic_desc

    def _gen_output_pic_desc(self, width, height,
                             output_buffer, output_buffer_size,
                             width_align_factor=16, height_align_factor=2):
        #创建图片输出desc
        stride_width = align_up(width, width_align_factor)
        stride_height = align_up(height, height_align_factor)

        pic_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(pic_desc, output_buffer)
        acl.media.dvpp_set_pic_desc_format(pic_desc,
                                           PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(pic_desc, width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(pic_desc, output_buffer_size)

        return pic_desc

    def _stride_yuv_size(self, width, height,
                         width_align_factor=16, height_align_factor=2):
        stride_width = align_up(width, width_align_factor)
        stride_height = align_up(height, height_align_factor)
        stride_size = yuv420sp_size(stride_width, stride_height)

        return stride_width, stride_height, stride_size


    def jpegd(self, image):
        #jepg图片转yuv图片
        #创建转换输出图片desc
        output_desc, out_buffer = self._gen_jpegd_out_pic_desc(image)
        ret = acl.media.dvpp_jpeg_decode_async(self._dvpp_channel_desc,
                                               image.data(),
                                               image.size,
                                               output_desc,
                                               self._stream)
        if ret != ACL_ERROR_NONE:
            print("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None

        ret = acl.rt.synchronize_stream(self._stream) 
        if ret != ACL_ERROR_NONE:
            print("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None     

        width, height, size = self._stride_yuv_size(image.width, image.height)
        return AclImage(out_buffer, width, height, size)

    def _gen_jpegd_out_pic_desc(self, image):
        #预测jpeg解码为yuv图片所需要的内存大小
        out_buffer_size, ret = acl.media.dvpp_jpeg_predict_dec_size( \
            image.data(), image.size, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        if ret != ACL_ERROR_NONE:
            print("Predict jpeg decode size failed, return ", ret)
            return None
        #申请存放解码后yuv图片的内存
        out_buffer, ret = acl.media.dvpp_malloc(out_buffer_size)
        if ret != ACL_ERROR_NONE:
            print("Dvpp malloc failed, error: ", ret)
            return None
        #创建输出图片desc
        pic_desc = self._gen_output_pic_desc(image.width, image.height,
                                             out_buffer, out_buffer_size)
        return pic_desc, out_buffer


    def resize(self, image, resize_width, resize_height):
        #缩放yuvsp420图片到指定大小
        #先生成输入图片desc
        input_desc = self._gen_input_pic_desc(image)
        #计算缩放后的图片尺寸
        stride_width = align_up16(resize_width)
        stride_height = align_up2(resize_height)
        output_size = yuv420sp_size(stride_width, stride_height)
        #为缩放后的图片申请内存
        out_buffer, ret = acl.media.dvpp_malloc(output_size)
        if ret != ACL_ERROR_NONE:
            print("Dvpp malloc failed, error: ", ret)
            return None
        #创建输出desc
        output_desc = self._gen_output_pic_desc(resize_width, resize_height,
                                                out_buffer, output_size)
        if output_desc == None:
            print("Gen resize output desc failed")
            return None
        #调用dvpp异步缩放接口缩放图片
        ret = acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                              input_desc,
                                              output_desc,
                                              self._resize_config,
                                              self._stream)
        if ret != ACL_ERROR_NONE:
            print("Vpc resize async failed, error: ", ret)
            return None
        #等待缩放操作完成
        ret = acl.rt.synchronize_stream(self._stream)
        if ret != ACL_ERROR_NONE:
            print("Resize synchronize stream failed, error: ", ret)
            return None
        #释放缩放申请的资源
        acl.media.dvpp_destroy_pic_desc(input_desc)
        acl.media.dvpp_destroy_pic_desc(output_desc)
        return AclImage(out_buffer, stride_width,
                        stride_height, output_size, MEMORY_DVPP)

    def _gen_resize_out_pic_desc(self, resize_width, 
                                 resize_height, output_size):                                 
        out_buffer, ret = acl.media.dvpp_malloc(output_size)
        if ret != ACL_ERROR_NONE:
            print("Dvpp malloc failed, error: ", ret)
            return None
        pic_desc = self._gen_output_pic_desc(resize_width, resize_height,
                                             out_buffer, output_size)
        return pic_desc, out_buffer


    def jpege(self, image):
        #将yuv420sp图片转为jpeg图片
        #创建输入图片desc
        input_desc = self._gen_input_pic_desc(image)
        #预测转换所需内存大小
        output_size, ret = acl.media.dvpp_jpeg_predict_enc_size(
            input_desc, self._jpege_config)
        if (ret != ACL_ERROR_NONE):
            print("Predict jpege output size failed")
            return None
        #申请转换需要的内存
        output_buffer, ret = acl.media.dvpp_malloc(output_size)
        if (ret != ACL_ERROR_NONE):
            print("Malloc jpege output memory failed")
            return None
        #输出大小参数为既是输入参数,也是输出参数,需要时一个指针
        output_size_array = np.array([output_size], dtype=np.int32)
        output_size_ptr = acl.util.numpy_to_ptr(output_size_array)

        #调用jpege异步接口转换图片
        ret = acl.media.dvpp_jpeg_encode_async(self._dvpp_channel_desc,
                                               input_desc, output_buffer,
                                               output_size_ptr,
                                               self._jpege_config,
                                               self._stream)
        if (ret != ACL_ERROR_NONE):
            print("Jpege failed, ret ", ret)
            return None
        #等待转换完成
        ret = acl.rt.synchronize_stream(self._stream)
        if (ret != ACL_ERROR_NONE):
            print("Jpege synchronize stream, failed, ret ", ret)
            return None
        #释放资源
        acl.media.dvpp_destroy_pic_desc(input_desc)
        return AclImage(output_buffer, image.width, 
                        image.height, int(output_size_array[0]), MEMORY_DVPP)
