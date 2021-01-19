import acl
from constants import *
from utils import *
from acl_image import AclImage


class Dvpp():
    def __init__(self, stream, run_model):
        self._stream = stream
        self._run_mode = run_model
        self._dvpp_channel_desc = None

    def __del__(self):
        if self._crop_config:
            acl.media.dvpp_destroy_roi_config(self._crop_config)
        if self._paste_config:
            acl.media.dvpp_destroy_roi_config(self._paste_config)
        if self._dvpp_channel_desc:
            acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)
        if self._jpege_config:
            acl.media.dvpp_destroy_jpege_config(self._jpege_config)

    def init_resource(self):
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        if ret != ACL_ERROR_NONE:
            print("Dvpp create channel failed")
            return FAILED
        self._jpege_config = acl.media.dvpp_create_jpege_config()
        return SUCCESS          

    def jpegd(self, image):
        print('[Dvpp] jpeg decode stage:')
        print("[Dvpp] jpeg width %d, height %d, size %d"%(image.width, image.height, image.size), " data ", image.data())
        output_desc, out_buffer = self._gen_jpegd_out_pic_desc(image)        
        image_dvpp = image.copy_to_dvpp(self._run_mode)
        ret = acl.media.dvpp_jpeg_decode_async(self._dvpp_channel_desc,
                                               image_dvpp.data(),
                                               image_dvpp.size,
                                               output_desc,
                                               self._stream)
        if ret != ACL_ERROR_NONE:
            print("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None

        ret = acl.rt.synchronize_stream(self._stream) 
        if ret != ACL_ERROR_NONE:
            print("dvpp_jpeg_decode_async failed ret={}".format(ret))
            return None     

        width = align_up128(image.width)
        height = align_up16(image.height)
        return AclImage(out_buffer, width, height, yuv420sp_size(width, height))                       

    def _gen_jpegd_out_pic_desc(self, image):
        stride_width = align_up128(image.width)
        stride_height = align_up16(image.height)

        out_buffer_size, ret = acl.media.dvpp_jpeg_predict_dec_size( \
            image.data(), image.size, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        if ret != ACL_ERROR_NONE:
            print("predict jpeg decode size failed, return ", ret)
            return None

        out_buffer, ret = acl.media.dvpp_malloc(out_buffer_size)
        check_ret("acl.media.dvpp_malloc", ret)

        pic_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(pic_desc, out_buffer)
        acl.media.dvpp_set_pic_desc_format(pic_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(pic_desc, image.width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, image.height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(pic_desc, out_buffer_size)

        return pic_desc, out_buffer                       

    def resize(self, image, resize_width, resize_height):
        print('[Dvpp] vpc resize stage:')
        input_desc = self._gen_input_pic_desc(image)        
        output_desc, out_buffer, out_buffer_size = \
            self._gen_resize_out_pic_desc(resize_width, resize_height)
        ret = acl.media.dvpp_vpc_resize_async(self._dvpp_channel_desc,
                                              input_desc,
                                              output_desc,
                                              self._resize_config,
                                              self._stream)
        check_ret("acl.media.dvpp_vpc_resize_async", ret)
        ret = acl.rt.synchronize_stream(self._stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] vpc resize stage success')

        stride_width = align_up16(image.width)
        stride_height = align_up2(image.height)
        return AclImage(out_buffer, stride_width, 
                        stride_height, out_buffer_size)

    def crop_and_paste(self, image, width, height, crop_and_paste_width, crop_and_paste_height):
        print('[Dvpp] vpc crop and paste stage:')
        input_desc = self._gen_input_pic_desc(image)
        output_desc, out_buffer, out_buffer_size = \
            self._gen_resize_out_pic_desc(crop_and_paste_width, crop_and_paste_height)
        self._crop_config = acl.media.dvpp_create_roi_config(0, (width >> 1 << 1) - 1, 0, (height >> 1 << 1) - 1)
        self._paste_config = acl.media.dvpp_create_roi_config(0, crop_and_paste_width - 1, 0, crop_and_paste_height - 1)
        ret = acl.media.dvpp_vpc_crop_and_paste_async(self._dvpp_channel_desc,
                                                      input_desc,
                                                      output_desc,
                                                      self._crop_config,
                                                      self._paste_config,
                                                      self._stream)
        check_ret("acl.media.dvpp_vpc_crop_and_paste_async", ret)
        ret = acl.rt.synchronize_stream(self._stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('[Dvpp] vpc crop and paste stage success')
        stride_width = align_up16(crop_and_paste_width)
        stride_height = align_up2(crop_and_paste_height)
        return AclImage(out_buffer, stride_width,
                        stride_height, out_buffer_size)

    def _gen_input_pic_desc(self, image, width_align=16, height_align=2):
        stride_width = align_up(image.width, align=width_align)
        stride_height = align_up(image.height, align=height_align)

        pic_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(pic_desc, image.data())
        acl.media.dvpp_set_pic_desc_format(pic_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(pic_desc, image.width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, image.height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(pic_desc, image.size)

        return pic_desc

    def _gen_resize_out_pic_desc(self, resize_width, resize_height):
        stride_width = align_up16(resize_width)
        stride_height = align_up2(resize_height)
        out_buffer_size = yuv420sp_size(stride_width, stride_height)
        out_buffer, ret = acl.media.dvpp_malloc(out_buffer_size)
        check_ret("acl.media.dvpp_malloc", ret)

        pic_desc = acl.media.dvpp_create_pic_desc()
        acl.media.dvpp_set_pic_desc_data(pic_desc, out_buffer)
        acl.media.dvpp_set_pic_desc_format(pic_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(pic_desc, resize_width)
        acl.media.dvpp_set_pic_desc_height(pic_desc, resize_height)
        acl.media.dvpp_set_pic_desc_width_stride(pic_desc, stride_width)
        acl.media.dvpp_set_pic_desc_height_stride(pic_desc, stride_height)
        acl.media.dvpp_set_pic_desc_size(pic_desc, out_buffer_size)

        return pic_desc, out_buffer, out_buffer_size

    def jpege(self, image, width_align=128, height_align=16):
        # 将yuv420sp图片转为jpeg图片
        # 创建输入图片desc
        input_desc = self._gen_input_pic_desc(image, width_align, height_align)
        # 预测转换所需内存大小
        output_size, ret = acl.media.dvpp_jpeg_predict_enc_size(
            input_desc, self._jpege_config)
        if (ret != ACL_ERROR_NONE):
            print("Predict jpege output size failed")
            return None
        # 申请转换需要的内存
        output_buffer, ret = acl.media.dvpp_malloc(output_size)
        if (ret != ACL_ERROR_NONE):
            print("Malloc jpege output memory failed")
            return None
        # 输出大小参数为既是输入参数,也是输出参数,需要时一个指针
        output_size_array = np.array([output_size], dtype=np.int32)
        output_size_ptr = acl.util.numpy_to_ptr(output_size_array)

        # 调用jpege异步接口转换图片
        ret = acl.media.dvpp_jpeg_encode_async(self._dvpp_channel_desc,
                                               input_desc, output_buffer,
                                               output_size_ptr,
                                               self._jpege_config,
                                               self._stream)
        if (ret != ACL_ERROR_NONE):
            print("Jpege failed, ret ", ret)
            return None
        # 等待转换完成
        ret = acl.rt.synchronize_stream(self._stream)
        if (ret != ACL_ERROR_NONE):
            print("Jpege synchronize stream failed, ret ", ret)
            return None

        # 释放资源
        acl.media.dvpp_destroy_pic_desc(input_desc)

        return AclImage(output_buffer, image.width,
                        image.height, int(output_size_array[0]), MEMORY_DVPP)
