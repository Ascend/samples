import os
import cv2
import numpy as np
import acl
from label import label

PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1
ACL_MEM_MALLOC_HUGE_FIRST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2
ALIGN128 = 128
ALIGN64 = 64
ALIGN16 = 16
ALIGN2 = 2
NPY_FLOAT32 = 11
ACL_SUCCESS = 0

class sample_resnet_dvpp(object):
    def __init__(self, device_id, model_path):
        self.device_id = device_id      # int
        self.context = None             # pointer
        self.stream = None

        self.dvpp_desc = None
        self.jpegd_output_desc = None
        self.resize_input_desc = None
        self.resize_output_desc = None
        self.resize_config = None
        self.jpegd_dvpp_buff = None
        self.jpegd_out_buff = None
        self.resize_output_buffer = None

        self.model_id = None            # pointer
        self.model_path = model_path    # string
        self.model_desc = None          # pointer when using

        self.input_dataset = None
        self.output_dataset = None

    def init_resource(self):
        # init acl resource
        ret = acl.init()
        if ret != ACL_SUCCESS:
            print('acl init failed, errorCode is', ret)
        ret = acl.rt.set_device(self.device_id)
        if ret != ACL_SUCCESS:
            print('set device failed, errorCode is', ret)
        self.context, ret = acl.rt.create_context(self.device_id)
        if ret != ACL_SUCCESS:
            print('set context failed, errorCode is', ret)
        self.stream, ret = acl.rt.create_stream()
        if ret != ACL_SUCCESS:
            print('set stream failed, errorCode is', ret)

        # create description of dvpp
        self.dvpp_desc = acl.media.dvpp_create_channel_desc()
        ret = acl.media.dvpp_create_channel(self.dvpp_desc)
        if ret != ACL_SUCCESS:
            print('dvpp create channel failed, errorCode is', ret)
        self.jpegd_output_desc = acl.media.dvpp_create_pic_desc()
        self.resize_input_desc = acl.media.dvpp_create_pic_desc()
        self.resize_output_desc = acl.media.dvpp_create_pic_desc()
        self.resize_config = acl.media.dvpp_create_resize_config()

        # load model from file
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        if ret != ACL_SUCCESS:
            print('load model failed, errorCode is', ret)

        # create description of model
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        if ret != ACL_SUCCESS:
            print('dvpp create channel failed, errorCode is', ret)

        # create data set of input
        model_input_size = acl.mdl.get_num_inputs(self.model_desc)
        self.input_dataset = acl.mdl.create_dataset()
        for i in range(model_input_size):
            input_buffer_size = acl.mdl.get_input_size_by_index(self.model_desc, i)
            input_buffer, ret = acl.rt.malloc(input_buffer_size,
                                              ACL_MEM_MALLOC_HUGE_FIRST)
            if ret != ACL_SUCCESS:
                print('create input_buffer failed, errorCode is', ret)
            data = acl.create_data_buffer(input_buffer, input_buffer_size)
            acl.mdl.add_dataset_buffer(self.input_dataset, data)

        # create data set of output
        model_output_size = acl.mdl.get_num_outputs(self.model_desc)
        self.output_dataset = acl.mdl.create_dataset()
        for i in range(model_output_size):
            output_buffer_size = acl.mdl.get_output_size_by_index(self.model_desc, i)
            output_buffer, ret = acl.rt.malloc(output_buffer_size,
                                               ACL_MEM_MALLOC_HUGE_FIRST)
            if ret != ACL_SUCCESS:
                print('create output_buffer failed, errorCode is', ret)
            data = acl.create_data_buffer(output_buffer, output_buffer_size)
            acl.mdl.add_dataset_buffer(self.output_dataset, data)

    def process_input(self, input_path, model_width, model_height):
        # read image from file
        src_image = np.fromfile(input_path, dtype=np.byte)
        jpegd_size = src_image.size * src_image.itemsize

        # get attributes of image
        cv_image = cv2.imread(input_path)
        height, width, _ = np.shape(cv_image)
        if 'bytes_to_ptr' in dir(acl.util):
            bytes_image = src_image.tobytes()
            data_ptr = acl.util.bytes_to_ptr(bytes_image)
        else:
            data_ptr = acl.util.numpy_to_ptr(src_image)

        # create dvpp malloc
        self.jpegd_dvpp_buff, ret = acl.media.dvpp_malloc(jpegd_size)
        if ret != ACL_SUCCESS:
            print('create dvpp buff failed, errorCode is', ret)

        # memory copy from host to dvpp
        ret = acl.rt.memcpy(self.jpegd_dvpp_buff, jpegd_size, data_ptr,
                            jpegd_size, ACL_MEMCPY_HOST_TO_DEVICE)
        if ret != ACL_SUCCESS:
            print('memcpy failed, errorCode is', ret)

        # alignment with different chip version
        soc_name = acl.get_soc_name()
        if "Ascend310P" in soc_name:
            jpegd_width = (width + ALIGN2 - 1) // ALIGN2 * ALIGN2
            jpegd_height = (height + ALIGN2 - 1) // ALIGN2 * ALIGN2
            jpegd_stride_width = (width + ALIGN64 - 1) // ALIGN64 * ALIGN64
            jpegd_stride_height = (height + ALIGN16 - 1) // ALIGN16 * ALIGN16
        else:
            jpegd_width = width
            jpegd_height = height
            jpegd_stride_width = (width + ALIGN128 - 1) // ALIGN128 * ALIGN128
            jpegd_stride_height = (height + ALIGN16 - 1) // ALIGN16 * ALIGN16

        # output size is calculated by (jpegd_stride_width * jpegd_stride_width) * 3 / 2
        jpegd_stride_size = int(jpegd_stride_width * jpegd_stride_height * 3 / 2)
        self.jpegd_out_buff, ret = acl.media.dvpp_malloc(jpegd_stride_size)
        if ret != ACL_SUCCESS:
            print('create dvpp buff failed, errorCode is', ret)

        # create output description and set the attributes
        acl.media.dvpp_set_pic_desc_data(self.jpegd_output_desc, self.jpegd_out_buff)
        acl.media.dvpp_set_pic_desc_format(self.jpegd_output_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(self.jpegd_output_desc, jpegd_width)
        acl.media.dvpp_set_pic_desc_height(self.jpegd_output_desc, jpegd_height)
        acl.media.dvpp_set_pic_desc_height_stride(self.jpegd_output_desc, jpegd_stride_height)
        acl.media.dvpp_set_pic_desc_width_stride(self.jpegd_output_desc, jpegd_stride_width)
        acl.media.dvpp_set_pic_desc_size(self.jpegd_output_desc, jpegd_stride_size)

        # decode image from JPEG format to YUV
        acl.media.dvpp_jpeg_decode_async(self.dvpp_desc, self.jpegd_dvpp_buff, jpegd_size,
                                         self.jpegd_output_desc, self.stream)
        ret = acl.rt.synchronize_stream(self.stream)
        if ret != ACL_SUCCESS:
            print('dvpp jpeg decode async failed, errorCode is', ret)

        # create input description and set the attributes
        acl.media.dvpp_set_pic_desc_data(self.resize_input_desc, self.jpegd_out_buff)
        acl.media.dvpp_set_pic_desc_format(self.resize_input_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(self.resize_input_desc, jpegd_width)
        acl.media.dvpp_set_pic_desc_height(self.resize_input_desc, jpegd_height)
        acl.media.dvpp_set_pic_desc_height_stride(self.resize_input_desc, jpegd_stride_height)
        acl.media.dvpp_set_pic_desc_width_stride(self.resize_input_desc, jpegd_stride_width)
        acl.media.dvpp_set_pic_desc_size(self.resize_input_desc, jpegd_stride_size)

        # output widthStride should be 16 aligned, heightStride should be 2 aligned
        resize_out_width = (model_width + ALIGN2 - 1) // ALIGN2 * ALIGN2
        resize_out_height = (model_height + ALIGN2 - 1) // ALIGN2 * ALIGN2
        resize_stride_width = (model_width + ALIGN16 - 1) // ALIGN16 * ALIGN16
        resize_stride_height = (model_height + ALIGN2 - 1) // ALIGN2 * ALIGN2

        # output size is calculated by (resize_stride_width * resize_stride_height) * 3 / 2
        resize_buffer_size = int((resize_stride_width * resize_stride_height) * 3 / 2)
        self.resize_output_buffer, ret = acl.media.dvpp_malloc(resize_buffer_size)
        if ret != ACL_SUCCESS:
            print('create dvpp buff failed, errorCode is', ret)

        # create output description and set the attributes
        acl.media.dvpp_set_pic_desc_data(self.resize_output_desc, self.resize_output_buffer)
        acl.media.dvpp_set_pic_desc_format(self.resize_output_desc, PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        acl.media.dvpp_set_pic_desc_width(self.resize_output_desc, resize_out_width)
        acl.media.dvpp_set_pic_desc_height(self.resize_output_desc, resize_out_height)
        acl.media.dvpp_set_pic_desc_height_stride(self.resize_output_desc, resize_stride_width)
        acl.media.dvpp_set_pic_desc_width_stride(self.resize_output_desc, resize_stride_height)
        acl.media.dvpp_set_pic_desc_size(self.resize_output_desc, resize_buffer_size)

        # execute resize
        acl.media.dvpp_vpc_resize_async(self.dvpp_desc,
                                        self.resize_input_desc,
                                        self.resize_output_desc,
                                        self.resize_config,
                                        self.stream)
        ret = acl.rt.synchronize_stream(self.stream)
        if ret != ACL_SUCCESS:
            print('dvpp vpc resize async failed, errorCode is', ret)

        # copy data to data buffer
        images = [self.resize_output_buffer]
        policy = ACL_MEMCPY_HOST_TO_DEVICE
        input_num = acl.mdl.get_dataset_num_buffers(self.input_dataset)
        for i in range(input_num):
            out_data_buffer = acl.mdl.get_dataset_buffer(self.input_dataset, i)
            out_data_buffer_addr = acl.get_data_buffer_addr(out_data_buffer)
            size = acl.get_data_buffer_size(out_data_buffer)
            ret = acl.rt.memcpy(out_data_buffer_addr,
                                size,
                                images[i],
                                size,
                                policy)
            if ret != ACL_SUCCESS:
                print('memcpy failed, errorCode is', ret)

    def inference(self):
        # inference
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        if ret != ACL_SUCCESS:
            print('execute model failed, errorCode is', ret)

    def get_result(self):
        # get result from output data set
        res = []
        output_index = 0
        out_data_buffer = acl.mdl.get_dataset_buffer(self.output_dataset, output_index)
        out_data_buffer_addr = acl.get_data_buffer_addr(out_data_buffer)
        size = acl.get_data_buffer_size(out_data_buffer)
        ptr, ret = acl.rt.malloc_host(size)
        if ret != ACL_SUCCESS:
            print('create host buff failed, errorCode is', ret)
        res.append({"size": size, "buffer": ptr})
        ret = acl.rt.memcpy(ptr,
                            size,
                            out_data_buffer_addr,
                            size,
                            ACL_MEMCPY_DEVICE_TO_HOST)
        if ret != ACL_SUCCESS:
            print('memcpy failed, errorCode is', ret)

        result = []
        out_index = 0
        dims, ret = acl.mdl.get_cur_output_dims(self.model_desc, out_index)
        if ret != ACL_SUCCESS:
            print('get output dims failed, errorCode is', ret)
        out_dim = dims['dims']
        for temp in res:
            ptr = temp["buffer"]
            if "ptr_to_bytes" in dir(acl.util):
                bytes_data = acl.util.ptr_to_bytes(ptr, temp["size"])
                data = np.frombuffer(bytes_data, dtype=np.float32).reshape(tuple(out_dim))
            else:
                data = acl.util.ptr_to_numpy(ptr, tuple(out_dim), NPY_FLOAT32)
            result.append(data)

        # do data processing with softmax
        vals = np.array(result).flatten()
        vals = np.exp(vals)/np.sum(np.exp(vals))

        # print top 5 classes
        top_k = vals.argsort()[-1:-6:-1]
        print("======== top5 inference results: =============")
        for j in top_k:
            label_class = ""
            label_string = label.get(str(j))
            if label_string:
                list_iterator = iter(range(len(label_string)))
                for i in list_iterator:
                    if i == len(label_string) - 1:
                        label_class += label_string[i]
                else:
                    label_class += label_string[i] + "," + "\t"
                print("label:%d confidence:%f class:%s" % (j, vals[j], label_class))
            else:
                raise Exception("the key of label is not exist")

        for item in res:
            ptr = item['buffer']
            ret = acl.rt.free_host(ptr)
            if ret != ACL_SUCCESS:
                print('free host failed, errorCode is', ret)

    def release_resource(self):
        ret = acl.media.dvpp_free(self.jpegd_dvpp_buff)
        if ret != ACL_SUCCESS:
            print('free dvpp malloc failed, errorCode is', ret)
        ret = acl.media.dvpp_free(self.jpegd_out_buff)
        if ret != ACL_SUCCESS:
            print('free dvpp malloc failed, errorCode is', ret)
        ret = acl.media.dvpp_destroy_pic_desc(self.jpegd_output_desc)
        if ret != ACL_SUCCESS:
            print('destroy description of picture failed, errorCode is', ret)

        ret = acl.media.dvpp_destroy_resize_config(self.resize_config)
        if ret != ACL_SUCCESS:
            print('destroy description of picture failed, errorCode is', ret)
        ret = acl.media.dvpp_destroy_pic_desc(self.resize_input_desc)
        if ret != ACL_SUCCESS:
            print('destroy description of picture failed, errorCode is', ret)
        ret = acl.media.dvpp_destroy_pic_desc(self.resize_output_desc)
        if ret != ACL_SUCCESS:
            print('destroy description of picture failed, errorCode is', ret)
        ret = acl.media.dvpp_free(self.resize_output_buffer)
        if ret != ACL_SUCCESS:
            print('free dvpp malloc failed, errorCode is', ret)

        ret = acl.media.dvpp_destroy_channel(self.dvpp_desc)
        if ret != ACL_SUCCESS:
            print('destroy  channel failed, errorCode is', ret)
        ret = acl.media.dvpp_destroy_channel_desc(self.dvpp_desc)
        if ret != ACL_SUCCESS:
            print('destroy description of channel failed, errorCode is', ret)

        # release resource includes acl resource, data set and unload model
        for dataset in [self.input_dataset, self.output_dataset]:
            if not dataset:
                continue
            number = acl.mdl.get_dataset_num_buffers(dataset)
            for i in range(number):
                data_buf = acl.mdl.get_dataset_buffer(dataset, i)
                if data_buf:
                    acl.destroy_data_buffer(data_buf)
            acl.mdl.destroy_dataset(dataset)
        ret = acl.mdl.unload(self.model_id)
        if ret != ACL_SUCCESS:
            print('unload model failed, errorCode is', ret)
        if self.model_desc:
            acl.mdl.destroy_desc(self.model_desc)
            self.model_desc = None

        if self.stream:
            ret = acl.rt.destroy_stream(self.stream)
            if ret != ACL_SUCCESS:
                print('destroy stream failed, errorCode is', ret)
        if self.context:
            ret = acl.rt.destroy_context(self.context)
            if ret != ACL_SUCCESS:
                print('destroy context failed, errorCode is', ret)
            self.context = None
        ret = acl.rt.reset_device(self.device_id)
        if ret != ACL_SUCCESS:
            print('reset device failed, errorCode is', ret)
        ret = acl.finalize()
        if ret != ACL_SUCCESS:
            print('finalize failed, errorCode is', ret)

if __name__ == '__main__':
    device = 0
    model_width = 224
    model_height = 224
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "../model/resnet50.om")
    if not os.path.exists(model_path):
        raise Exception("the model is not exist")

    # read all file path of image
    images_path = os.path.join(current_dir, "../data")
    if not os.path.exists(images_path):
        raise Exception("the directory is not exist")
    all_path = []
    for path in os.listdir(images_path):
        if path != '.keep':
            total_path = os.path.join(images_path, path)
            all_path.append(total_path)
    if len(all_path) == 0:
        raise Exception("the directory is empty, please download image")

    net = sample_resnet_dvpp(device, model_path)
    net.init_resource()
    for path in all_path:
        net.process_input(path, model_width, model_height)
        net.inference()
        net.get_result()
    print("*****run finish******")
    net.release_resource()
