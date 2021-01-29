import acl
from constants import *
from utils import *
from acl_image import AclImage

class Dvpp():
    def __init__(self, stream, run_model):
        self._stream = stream
        self._run_mode = run_model
        self._dvpp_channel_desc = None
        self._resize_config = None

    def __del__(self):
        if self._resize_config:
            acl.media.dvpp_destroy_resize_config(self._resize_config)
        if self._dvpp_channel_desc:
            acl.media.dvpp_destroy_channel(self._dvpp_channel_desc)
            acl.media.dvpp_destroy_channel_desc(self._dvpp_channel_desc)

    def init_resource(self):
        self._dvpp_channel_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self._dvpp_channel_desc)
        if ret != ACL_ERROR_NONE:
            print("Dvpp create channel failed")
            return FAILED
        self._resize_config = acl.media.dvpp_create_resize_config()

        return SUCCESS          

    def jpegd(self, image):
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
        return AclImage(out_buffer, width, height, yuv420sp_size(width, height), MEMORY_DEVICE)

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
                        stride_height, out_buffer_size, MEMORY_DEVICE)

    def _gen_input_pic_desc(self, image):
        stride_width = align_up128(image.width)
        stride_height = align_up16(image.height) 
 
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
