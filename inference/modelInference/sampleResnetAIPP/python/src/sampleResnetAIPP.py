import os
import numpy as np
import cv2
import acl
from label import label

ACL_MEM_MALLOC_HUGE_FIRST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2
NPY_FLOAT32 = 11
ACL_SUCCESS = 0

class sample_resnet_aipp(object):
    def __init__(self, device_id, model_path):
        self.device_id = device_id      # int
        self.context = None             # pointer
        self.stream = None
        
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
            print('create context failed, errorCode is', ret)
        self.stream, ret = acl.rt.create_stream()
        if ret != ACL_SUCCESS:
            print('create stream failed, errorCode is', ret)

        # load model from file
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)

        # create model description
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        if ret != ACL_SUCCESS:
            print('get desc failed, errorCode is', ret)

        # create input data set
        input_size = acl.mdl.get_num_inputs(self.model_desc)
        self.input_dataset = acl.mdl.create_dataset()
        for i in range(input_size):
            input_buffer_size = acl.mdl.get_input_size_by_index(self.model_desc, i)
            input_buffer, ret = acl.rt.malloc(input_buffer_size,
                                              ACL_MEM_MALLOC_HUGE_FIRST)
            if ret != ACL_SUCCESS:
                print('create input_buffer failed, errorCode is', ret)
            input_data_buffer = acl.create_data_buffer(input_buffer, input_buffer_size)
            acl.mdl.add_dataset_buffer(self.input_dataset, input_data_buffer)

        # create output data set
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        self.output_dataset = acl.mdl.create_dataset()
        for i in range(output_size):
            output_buffer_size = acl.mdl.get_output_size_by_index(self.model_desc, i)
            output_buffer, ret = acl.rt.malloc(output_buffer_size,
                                               ACL_MEM_MALLOC_HUGE_FIRST)

            output_data_buffer = acl.create_data_buffer(output_buffer, output_buffer_size)
            acl.mdl.add_dataset_buffer(self.output_dataset, output_data_buffer)

    def process_input(self, input_path, model_width, model_height):
        # read image from file by opencv
        input_path = os.path.abspath(input_path)
        image = cv2.imread(input_path)

        # zoom image to model_width * model_height
        resize_image = cv2.resize(image, (model_width, model_height))

        # switch channel BGR to RGB
        RGB_image = cv2.cvtColor(resize_image, cv2.COLOR_BGR2RGB)
        RGB_image = RGB_image.astype("uint8")
        result = np.frombuffer(RGB_image.tobytes(), np.uint8)
        images = [result]

        # pass image data to input data buffer
        policy = ACL_MEMCPY_HOST_TO_DEVICE
        input_num = acl.mdl.get_dataset_num_buffers(self.input_dataset)
        for i in range(input_num):
            if "bytes_to_ptr" in dir(acl.util):
                bytes_data = images[i].tobytes()
                ptr = acl.util.bytes_to_ptr(bytes_data)
            else:
                ptr = acl.util.numpy_to_ptr(images[i])
            input_data_buffer = acl.mdl.get_dataset_buffer(self.input_dataset, i)
            input_data_buffer_addr = acl.get_data_buffer_addr(input_data_buffer)
            input_data_size = acl.get_data_buffer_size(input_data_buffer)
            ret = acl.rt.memcpy(input_data_buffer_addr,
                                input_data_size,
                                ptr,
                                input_data_size,
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
        output_data_buffer = acl.mdl.get_dataset_buffer(self.output_dataset, output_index)
        output_data_buffer_addr = acl.get_data_buffer_addr(output_data_buffer)
        output_data_size = acl.get_data_buffer_size(output_data_buffer)
        ptr, ret = acl.rt.malloc_host(output_data_size)
        res.append({"size": output_data_size, "buffer": ptr})
        ret = acl.rt.memcpy(ptr,
                            output_data_size,
                            output_data_buffer_addr,
                            output_data_size,
                            ACL_MEMCPY_DEVICE_TO_HOST)
        if ret != ACL_SUCCESS:
            print('memcpy failed, errorCode is', ret)

        result = []
        out_index = 0
        dims, ret = acl.mdl.get_cur_output_dims(self.model_desc, out_index)
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

    # inference
    net = sample_resnet_aipp(device, model_path)
    net.init_resource()
    for image in all_path:
        net.process_input(image, model_width, model_height)
        net.inference()
        net.get_result()
    print("*****run finish******")
    net.release_resource()
