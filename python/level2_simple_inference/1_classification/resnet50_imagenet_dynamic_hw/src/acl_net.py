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
import argparse
import numpy as np
import acl
import os
from PIL import Image
from constant import ACL_MEM_MALLOC_HUGE_FIRST, \
    ACL_MEMCPY_HOST_TO_DEVICE, ACL_MEMCPY_DEVICE_TO_HOST, \
    ACL_SUCCESS, IMG_EXT, NPY_FLOAT32, ACL_ERROR_GE_SHAPE_INVALID

buffer_method = {
    "in": acl.mdl.get_input_size_by_index,
    "out": acl.mdl.get_output_size_by_index
    }


def check_ret(message, ret):
    if ret != ACL_SUCCESS:
        raise Exception("{} failed ret={}"
                        .format(message, ret))


class Net(object):
    def __init__(self, device_id, model_path):
        self.device_id = device_id      # int
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.context = None             # pointer

        self.input_data = []
        self.output_data = []
        self.model_desc = None          # pointer when using
        self.load_input_dataset = None
        self.load_output_dataset = None
        self.height = None
        self.width = None

        self.init_resource()
    #释放资源
    def release_resource(self):
        print("Releasing resources stage:")
        ret = acl.mdl.unload(self.model_id)
        check_ret("acl.mdl.unload", ret)
        if self.model_desc:
            acl.mdl.destroy_desc(self.model_desc)
            self.model_desc = None

        while self.input_data:
            item = self.input_data.pop()
            ret = acl.rt.free(item["buffer"])
            check_ret("acl.rt.free", ret)

        while self.output_data:
            item = self.output_data.pop()
            ret = acl.rt.free(item["buffer"])
            check_ret("acl.rt.free", ret)

        if self.context:
            ret = acl.rt.destroy_context(self.context)
            check_ret("acl.rt.destroy_context", ret)
            self.context = None

        ret = acl.rt.reset_device(self.device_id)
        check_ret("acl.rt.reset_device", ret)
        ret = acl.finalize()
        check_ret("acl.finalize", ret)
        print('Resources released successfully.')
    #初始化
    def init_resource(self):
        print("init resource stage:")
        ret = acl.init()
        check_ret("acl.init", ret)
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        # load_model
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        check_ret("acl.mdl.load_from_file", ret)
        print("model_id:{}".format(self.model_id))

        self.model_desc = acl.mdl.create_desc()
        self._get_model_info()
        print("init resource success")
    #获取模型信息
    def _get_model_info(self,):
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        check_ret("acl.mdl.get_desc", ret)
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        input_size = acl.mdl.get_num_inputs(self.model_desc)
        self._gen_data_buffer(input_size, des="in")
        self._gen_data_buffer(output_size, des="out")
    #申请内存
    def _gen_data_buffer(self, size, des):
        func = buffer_method[des]
        for i in range(size):
            # check temp_buffer dtype
            temp_buffer_size = func(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             ACL_MEM_MALLOC_HUGE_FIRST)
            check_ret("acl.rt.malloc", ret)

            if des == "in":
                self.input_data.append({"buffer": temp_buffer,
                                        "size": temp_buffer_size})
            elif des == "out":
                self.output_data.append({"buffer": temp_buffer,
                                         "size": temp_buffer_size})

    def _data_interaction(self, dataset, policy=ACL_MEMCPY_HOST_TO_DEVICE):
        temp_data_buffer = [self.input_data[0]] \
            if policy == ACL_MEMCPY_HOST_TO_DEVICE \
            else self.output_data
        if len(dataset) == 0 and policy == ACL_MEMCPY_DEVICE_TO_HOST:
            for item in self.output_data:
                temp, ret = acl.rt.malloc_host(item["size"])
                if ret != 0:
                    raise Exception("can't malloc_host ret={}".format(ret))
                dataset.append({"size": item["size"], "buffer": temp})

        for i, item in enumerate(temp_data_buffer):
            if policy == ACL_MEMCPY_HOST_TO_DEVICE:
                if "bytes_to_ptr" in dir(acl.util):
                    bytes_data = dataset[i].tobytes()
                    ptr = acl.util.bytes_to_ptr(bytes_data)
                else:
                    ptr = acl.util.numpy_to_ptr(dataset[i])
                ret = acl.rt.memcpy(item["buffer"],
                                    item["size"],
                                    ptr,
                                    item["size"],
                                    policy)
                check_ret("acl.rt.memcpy", ret)

            else:
                ptr = dataset[i]["buffer"]
                ret = acl.rt.memcpy(ptr,
                                    item["size"],
                                    item["buffer"],
                                    item["size"],
                                    policy)
                check_ret("acl.rt.memcpy", ret)

    def _gen_dataset(self, type_str="in"):
        dataset = acl.mdl.create_dataset()
        temp_dataset = None
        if type_str == "in":
            self.load_input_dataset = dataset
            temp_dataset = self.input_data
        else:
            self.load_output_dataset = dataset
            temp_dataset = self.output_data
        
        for item in temp_dataset:    
            data = acl.create_data_buffer(item["buffer"], item["size"])

            _, ret = acl.mdl.add_dataset_buffer(dataset, data)

            if ret != ACL_SUCCESS:
                ret = acl.destroy_data_buffer(data)
                check_ret("acl.destroy_data_buffer", ret)

    def _data_from_host_to_device(self, images):
        print("data interaction from host to device")
        # copy images to device
        self._data_interaction(images, ACL_MEMCPY_HOST_TO_DEVICE)
        # load input data into model
        self._gen_dataset("in")
        # load output data into model
        self._gen_dataset("out")
        print("data interaction from host to device success")

    def _data_from_device_to_host(self):
        print("data interaction from device to host")
        res = []
        # copy device to host
        self._data_interaction(res, ACL_MEMCPY_DEVICE_TO_HOST)
        print("data interaction from device to host success")
        result = self.get_result(res)
        self._print_result(result)
        # free host memory
        for item in res:
            ptr = item['buffer']
            ret = acl.rt.free_host(ptr)
            check_ret('acl.rt.free_host', ret)

    def run(self, images):
        self._data_from_host_to_device(images)
        print("after forward")
        self.forward()
        self._data_from_device_to_host()

    #自定义函数，设置动态分辨率
    def _model_set_dynamicInfo(self):
        # 获取动态分辨率输入的index，标识动态分辨率输入的输入名称固定为"ascend_mbatch_shape_data"
        index, ret = acl.mdl.get_input_index_by_name(self.model_desc, "ascend_mbatch_shape_data")
        height = self.height
        width = self.width
        check_ret("get_input_index_by_name", ret)
        # 设置输入图片分辨率，model_id表示加载成功的模型的ID，input表示aclmdlDataset类型的数据，index表示标识动态分辨率输入的输入index
        hw_info,ret = acl.mdl.get_dynamic_hw(self.model_desc,index)
        check_ret("acl.mdl.get_dynamic_hw", ret)
        if [height, width] not in hw_info["hw"]:
            ret = ACL_ERROR_GE_SHAPE_INVALID 
            check_ret("[dynamic hw] {} is not in {}".format([height, width], hw_info["hw"]), ret)
        
        ret = acl.mdl.set_dynamic_hw_size(self.model_id, self.load_input_dataset, index, height, width)
        check_ret("acl.mdl.set_dynamic_hw_size", ret) 
        
        return ret

    def forward(self):
        print('execute stage:')
        #4.1 调用自定义函数，设置动态分辨率
        ret = self._model_set_dynamicInfo()
        check_ret("_model_set_dynamicInfo", ret)

        ret = acl.mdl.execute(self.model_id,
                              self.load_input_dataset,
                              self.load_output_dataset)

        check_ret("acl.mdl.execute", ret)
        self._destroy_databuffer()
        print('execute stage success')

    def _print_result(self, result):
        vals = np.array(result).flatten()
        vals = np.exp(vals)/np.sum(np.exp(vals))
        top_k = vals.argsort()[-1:-6:-1]
        print("======== top5 inference results: =============")
        for j in top_k:
            print("[%d]: %f" % (j, vals[j]))

    def _destroy_databuffer(self):
        for dataset in [self.load_input_dataset, self.load_output_dataset]:
            if not dataset:
                continue
            number = acl.mdl.get_dataset_num_buffers(dataset)
            for i in range(number):
                data_buf = acl.mdl.get_dataset_buffer(dataset, i)
                if data_buf:
                    ret = acl.destroy_data_buffer(data_buf)
                    check_ret("acl.destroy_data_buffer", ret)
            ret = acl.mdl.destroy_dataset(dataset)
            check_ret("acl.mdl.destroy_dataset", ret)

    def get_result(self, output_data):
        result = []
        dims, ret = acl.mdl.get_cur_output_dims(self.model_desc, 0)
        check_ret("acl.mdl.get_cur_output_dims", ret)
        out_dim = dims['dims']
        for temp in output_data:
            ptr = temp["buffer"]
            # 转化为float32类型的数据
            if "ptr_to_bytes" in dir(acl.util):
                bytes_data = acl.util.ptr_to_bytes(ptr, temp["size"])
                data = np.frombuffer(bytes_data, dtype=np.float32).reshape(tuple(out_dim))
            else:
                data = acl.util.ptr_to_numpy(ptr, tuple(out_dim), NPY_FLOAT32)
            result.append(data)
        return result

    def transfer_pic(self, input_path,h,w):
        self.height = h
        self.width = w
        input_path = os.path.abspath(input_path)
        with Image.open(input_path) as image_file:
            height = image_file.height
            width = image_file.width
            # 对图片进行切分，取中间区域
            h_off = height // 10
            w_off = width // 10
            box = (h_off, w_off, height - h_off, width - w_off)
            crop_img = image_file.crop(box)
            img = crop_img.resize((h, w))
            img = np.array(img)

        shape = img.shape
        img = img.astype("uint8")
        img = img.reshape([1] + list(shape))
        result = np.frombuffer(img.tobytes(), np.uint8)
        return result

if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument('--h', type=int, default=224)
    parser.add_argument('--w', type=int, default=224)
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--model_path', type=str,
                        default=os.path.join(current_dir, "../model/tf_resnet50.om"))
    parser.add_argument('--images_path', type=str,
                        default=os.path.join(current_dir, "../data"))
    args = parser.parse_args()
    print("Using device id:{}\nmodel path:{}\nimages path:{}"
          .format(args.device, args.model_path, args.images_path))

    net = Net(args.device, args.model_path)
    images_list = [os.path.join(args.images_path, img)
                   for img in os.listdir(args.images_path)
                   if os.path.splitext(img)[1] in IMG_EXT]

    for image in images_list:
        print("images:{}".format(image))
        img = net.transfer_pic(image,args.h,args.w)
        net.run([img])

    print("*****run finish******")
    net.release_resource()
