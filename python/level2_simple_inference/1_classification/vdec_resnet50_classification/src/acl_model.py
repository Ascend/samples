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
from constant import ACL_MEM_MALLOC_NORMAL_ONLY, \
    ACL_MEMCPY_DEVICE_TO_HOST, NPY_FLOAT32
from acl_util import check_ret


class Model(object):
    def __init__(self,
                 context,
                 stream,
                 model_path):
        self.model_path = model_path  # string
        self.model_id = None  # pointer
        self.context = context  # pointer
        self.stream = stream
        self.device_outputs = []
        self.input_data = None
        self.output_data = None
        self.model_desc = None  # pointer when using

        self.init_resource()

    def init_resource(self):
        print("[Model] class Model init resource stage:")
        acl.rt.set_context(self.context)
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        check_ret("acl.mdl.get_desc", ret)
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        self._gen_output_dataset(output_size)
        print("[Model] class Model init resource stage success")

    def __del__(self):
        self._release_dataset()
        while self.device_outputs:
            ret = acl.rt.free(self.device_outputs.pop())
            check_ret("acl.rt.free", ret)
        if self.model_id:
            ret = acl.mdl.unload(self.model_id)
            check_ret("acl.mdl.unload", ret)
        if self.model_desc:
            ret = acl.mdl.destroy_desc(self.model_desc)
            check_ret("acl.mdl.destroy_desc", ret)
        print("[Model] class Model release source success")

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            temp_buffer_size = acl.mdl.\
                get_output_size_by_index(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            self.device_outputs.append(temp_buffer)
            dataset_buffer = acl.create_data_buffer(
                temp_buffer,
                temp_buffer_size)

            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                ret = acl.destroy_data_buffer(dataset_buffer)
                check_ret("acl.destroy_data_buffer", ret)
        self.output_data = dataset
        print("[Model] create model output dataset success")

    def run(self, dvpp_output_buffer, dvpp_output_size):
        self._gen_input_dataset(dvpp_output_buffer, dvpp_output_size)
        self.forward()
        self._print_result(self.output_data)

    def forward(self):
        print('[Model] execute stage:')
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_data)
        check_ret("acl.mdl.execute", ret)
        print('[Model] execute stage success')

    def _gen_input_dataset(self, dvpp_output_buffer, dvpp_output_size):
        print("[Model] create model input dataset:")
        self.input_dataset = acl.mdl.create_dataset()
        input_dataset_buffer = acl.create_data_buffer(dvpp_output_buffer,
                                                      dvpp_output_size)
        _, ret = acl.mdl.add_dataset_buffer(
            self.input_dataset,
            input_dataset_buffer)
        if ret:
            ret = acl.destroy_data_buffer(input_dataset_buffer)
            check_ret("acl.destroy_data_buffer", ret)
        print("[Model] create model input dataset success")

    def _print_result(self, infer_output):
        num = acl.mdl.get_dataset_num_buffers(infer_output)
        for i in range(num):
            dims, ret = acl.mdl.get_cur_output_dims(self.model_desc, i)
            check_ret("acl.mdl.get_cur_output_dims", ret)
            out_dim = dims['dims']
            temp_output_buffer = acl.mdl.get_dataset_buffer(infer_output, i)

            infer_output_ptr = acl.get_data_buffer_addr(temp_output_buffer)
            infer_output_size = acl.get_data_buffer_size_v2(temp_output_buffer)

            output_host, _ = acl.rt.malloc_host(infer_output_size)
            acl.rt.memcpy(output_host, infer_output_size, infer_output_ptr,
                          infer_output_size, ACL_MEMCPY_DEVICE_TO_HOST)
            if "ptr_to_bytes" in dir(acl.util):
                bytes_data = acl.util.ptr_to_bytes(output_host, infer_output_size)
                result = np.frombuffer(bytes_data, dtype=np.float32).reshape(tuple(out_dim))
            else:
                result = acl.util.ptr_to_numpy(output_host,
                                            tuple(out_dim),
                                            NPY_FLOAT32)
            vals = np.array(result).flatten()
            top_k = vals.argsort()[-1:-6:-1]
            possible = 0
            print("\n========= top5 inference results: =========")
            for j in top_k:
                print("label:%d  prob: %f" % (j, vals[j]))
                possible += vals[j]
            print("result: class_label[{}],top1[{:f}],top5[{:f}] "
                  .format(top_k[0], vals[top_k[0]],
                  possible if possible < 1 else 1))
            ret = acl.rt.free_host(output_host)
            check_ret("acl.rt.free_host", ret)

    def _release_dataset(self):
        for dataset in [self.input_dataset, self.output_data]:
            if not dataset:
                continue
            num_buffers = acl.mdl.get_dataset_num_buffers(dataset)
            for i in range(num_buffers):
                data_buf = acl.mdl.get_dataset_buffer(dataset, i)
                if data_buf:
                    ret = acl.destroy_data_buffer(data_buf)
                    check_ret("acl.destroy_data_buffer", ret)
            ret = acl.mdl.destroy_dataset(dataset)
            check_ret("acl.mdl.destroy_dataset", ret)
