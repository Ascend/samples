import numpy as np

import acl

from atlas_utils.utils import *
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_logger import log_error, log_info
from atlas_utils.resource_list import resource_list

class Dvpp():
    def __init__(self, acl_resource=None):
        if acl_resource is None:
            self._stream, ret = acl.rt.create_stream()
            check_ret("acl.rt.create_stream", ret)
            self._run_mode, ret = acl.rt.get_run_mode()
            check_ret("acl.rt.get_run_mode", ret)
        else:
            self._stream = acl_resource.stream
            self._run_mode = acl_resource.run_mode
        self._dvpp_channel_desc = None
        self._init_resource()

        #dvpp涉及acl资源,程序退出时需要在acl结束前释放,
        #这里注册到资源表以确保释放时序
        self._is_destroyed = False
        resource_list.register(self)

    def _init_resource(self):
        #创建dvpp channel
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        check_ret("acl.media.dvpp_create_channel", ret)

        #创建resize配置
        self._resize_config = acl.media.dvpp_create_resize_config()

        #创建yuv转jpeg的配置
        self._jpege_config = acl.media.dvpp_create_jpege_config()
        ret = acl.media.dvpp_set_jpege_config_level(self._jpege_config, 100)
        check_ret("acl.media.dvpp_set_jpege_config_level", ret)


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
        '''
        jepg图片转yuv图片
        '''
        #创建转换输出图片desc
        output_desc, out_buffer = self._gen_jpegd_out_pic_desc(image)
        ret = acl.media.dvpp_jpeg_decode_async(self._dvpp_channel_desc,
                                               image.data(),
                                               image.size,
                                               output_desc,
                                               self._stream)
        if ret != ACL_ERROR_NONE:
            log_error("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None

        ret = acl.rt.synchronize_stream(self._stream) 
        if ret != ACL_ERROR_NONE:
            log_error("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None     

        #返回解码后的AclImage实例
        stride_width = align_up16(width)
        stride_height = align_up2(height)
        stride_size = yuv420sp_size(stride_width, stride_height)
        return AclImage(out_buffer, stride_width, 
                        stride_height, stride_size, MEMORY_DVPP)

    def _gen_jpegd_out_pic_desc(self, image):
        #预测jpeg解码为yuv图片所需要的内存大小
        ret, size = self._get_jpegd_memory_size(image)
        if ret == False:
            return None
        #申请存放解码后yuv图片的内存
        out_buffer, ret = acl.media.dvpp_malloc(out_buffer_size)
        if ret != ACL_ERROR_NONE:
            log_error("Dvpp malloc failed, error: ", ret)
            return None
        #创建输出图片desc
        pic_desc = self._gen_output_pic_desc(image.width, image.height,
                                             out_buffer, out_buffer_size)
        return pic_desc, out_buffer

    def _get_jpegd_memory_size(self, image):
        #dvpp_jpeg_predict_dec_size要求图片在host侧,对于atlas200dk来说,host和device
        #是一体的,任意内存都满足.但是atlas300上只有host侧内存才行
        if image.is_local():
            size, ret = acl.media.dvpp_jpeg_predict_dec_size( \
                image.data(), image.size, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
            if ret != ACL_ERROR_NONE:
                log_error("Predict jpeg decode size failed, return ", ret)
                return False, 0
            return True, size
        else:
            #如果图片内存不满足条件,直接给一片足够大的内存以避免图片从device拷贝到host拷贝
            return True, int(yuv420sp_size(image.width, image.height) * 3)

    def resize(self, image, resize_width, resize_height):
        '''
        缩放yuvsp420图片到指定大小
        '''
        #先生成输入图片desc
        input_desc = self._gen_input_pic_desc(image)
        #计算缩放后的图片尺寸
        stride_width = align_up16(resize_width)
        stride_height = align_up2(resize_height)
        output_size = yuv420sp_size(stride_width, stride_height)
        #为缩放后的图片申请内存
        out_buffer, ret = acl.media.dvpp_malloc(output_size)
        if ret != ACL_ERROR_NONE:
            log_error("Dvpp malloc failed, error: ", ret)
            return None
        #创建输出desc
        output_desc = self._gen_output_pic_desc(resize_width, resize_height,
                                                out_buffer, output_size)
        if output_desc == None:
            log_error("Gen resize output desc failed")
            return None
        #调用dvpp异步缩放接口缩放图片
        ret = acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                              input_desc,
                                              output_desc,
                                              self._resize_config,
                                              self._stream)
        if ret != ACL_ERROR_NONE:
            log_error("Vpc resize async failed, error: ", ret)
            return None
        #等待缩放操作完成
        ret = acl.rt.synchronize_stream(self._stream)
        if ret != ACL_ERROR_NONE:
            log_error("Resize synchronize stream failed, error: ", ret)
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
            log_error("Dvpp malloc failed, error: ", ret)
            return None
        pic_desc = self._gen_output_pic_desc(resize_width, resize_height,
                                             out_buffer, output_size)
        return pic_desc, out_buffer


    def jpege(self, image):
        '''
        将yuv420sp图片转为jpeg图片
        '''
        #创建输入图片desc
        input_desc = self._gen_input_pic_desc(image)
        #预测转换所需内存大小
        output_size, ret = acl.media.dvpp_jpeg_predict_enc_size(
            input_desc, self._jpege_config)
        if (ret != ACL_ERROR_NONE):
            log_error("Predict jpege output size failed")
            return None
        #申请转换需要的内存
        output_buffer, ret = acl.media.dvpp_malloc(output_size)
        if (ret != ACL_ERROR_NONE):
            log_error("Malloc jpege output memory failed")
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
            log_error("Jpege failed, ret ", ret)
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

    def destroy(self):
        if self._is_destroyed:
            return

        if self._resize_config:
            acl.media.dvpp_destroy_resize_config(self._resize_config)

        if self._dvpp_channel_desc:
            acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)

        if self._jpege_config:
            acl.media.dvpp_destroy_jpege_config(self._jpege_config)
        #dvpp资源释放完后修改资源表中对应状态,防止重复释放
        self._is_destroyed = True
        resource_list.unregister(self)
        log_info("dvpp resource release success")

    def __del__(self):
        self.destroy()