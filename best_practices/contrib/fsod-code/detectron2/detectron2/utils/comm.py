# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
This file contains primitives for multi-gpu communication.
This is useful when doing distributed training.
"""

import functools
import logging
import numpy as np
import pickle
import torch
import os
import torch.distributed as dist
from detectron2.structures import Boxes
from detectron2.structures import Instances

_LOCAL_PROCESS_GROUP = None
"""
A torch process group which only includes processes that on the same machine as the current process.
This variable is set when processes are spawned by `launch()` in "engine/launch.py".
"""

def set_device(obj, device='cpu'):
    if isinstance(obj, (tuple, list)):
        dump = []
        for item in obj:
            dump.append(set_device(item, device))
        return dump
    elif isinstance(obj, dict):
        dump = {}
        for key, value in obj.items():
            dump[key] = set_device(value, device)
        return dump
    elif isinstance(obj, Boxes):
        dump = Boxes(obj.tensor.to(device))
        return dump
    elif isinstance(obj, Instances):
        dump = Instances(obj._image_size)
        for key, value in obj._fields.items():
            dump.set(key, set_device(value, device))
        return dump
    elif isinstance(obj, torch.Tensor):
        return obj.to(device)
    else:
        return obj


def dump_tensor(output, name):
    dump = set_device(output, 'cpu')
    torch.save(dump,name)
    print('%s dump success!' %(name))


def load_tensor(name, device):
    output = torch.load(name)
    dump = set_device(output, device)
    print('%s load success!' % (name))
    return dump


def pres_check(gpu_path, npu_path):
    gpu_list = []
    npu_list = []
    file_list = os.listdir(npu_path)
    for item in file_list:
        if item.endswith('npu.dat'):
            npu_list.append(os.path.join(npu_path, item))
            gpu_list.append(os.path.join(gpu_path, item[:-7] + 'gpu.dat'))
    print('all compare file:', gpu_list)

    for npu_item,gpu_item in zip(npu_list, gpu_list):
        print('start check %s and %s'%(npu_item, gpu_item))
        gpu_data = torch.load(gpu_item)
        npu_data = torch.load(npu_item)
        pres_check_item(gpu_data, npu_data, gpu_item)


def pres_check_item(gpu_data, npu_data, gpu_item):
    assert(type(gpu_data) == type(npu_data))
    if isinstance(gpu_data, (tuple, list)):
        for g_item, n_item in zip(gpu_data, npu_data):
            pres_check_item(g_item, n_item, gpu_item)
    elif isinstance(gpu_data, dict):
        for key, val in gpu_data.items():
            pres_check_item(val, npu_data[key], gpu_item + '_' + key)
    elif isinstance(gpu_data, Boxes):
        pres_check_item(gpu_data.tensor, npu_data.tensor, gpu_item)
    elif isinstance(gpu_data, Instances):
        for key, val in gpu_data._fields.items():
            pres_check_item(val, npu_data._fields[key], gpu_item + '_' + key)
    elif isinstance(gpu_data, torch.Tensor):
        g_np = gpu_data.detach().numpy()
        n_np = npu_data.detach().numpy()
        compare_res(g_np, n_np, os.path.basename(gpu_item))

def compare_res(x, y, testcase_name, prec=None, prec16=None):
    # pytorch guixiaobing
    threshold = 1.e-4
    threshold2 = 1.e-3
    if prec is None:
        prec = threshold
    if prec16 is None:
        prec16 = threshold2
    size = x.size
    if torch.is_tensor(x) and torch.is_tensor(y):
        x = x.numpy()
        y = y.numpy()
    if (x.shape != y.shape):
        print("%s shpae error"%(testcase_name))
        return
    if (x.dtype != y.dtype):
        if(x.dtype == np.int8) or (x.dtype == np.int64):
            x = np.int32(x)
        else:
            print("%s dtype error, %s, %s"%(testcase_name, x.dtype, y.dtype))
            return
    dtype_list = [np.bool, np.int32, np.float16, np.float32]
    if x.dtype not in dtype_list:
        print("%s required dtype in [np.bool, np.int32, np.float16, np.float32]"%(testcase_name))
        return
    if x.dtype == np.bool:
        result = np.equal(x, y)
        if result.all() == False:
            print("%s error" % testcase_name)
            return
    elif (x.dtype == np.int32):
        result = np.equal(x, y)
        err_cnt = size-result.sum()
        if result.all() == False:
            print("%s error, err_cnt: %d, all_cnt: %d, err_ratio: %f" %(testcase_name,err_cnt, size, float(err_cnt)/size))
            return
    elif (x.dtype == np.float16):
        result = np.abs(y - x)
        result = np.less_equal(result, prec16 * np.abs(x))
        err_cnt = np.sum(result == False)
        if result.all() == False:
            if err_cnt > size * prec16:
                print("%s error, err_cnt: %d, all_cnt: %d, err_ratio: %f" %(testcase_name,err_cnt, size, float(err_cnt)/size))
                return
    elif (x.dtype == np.float32):
        result = np.abs(y - x)
        result = np.less_equal(result, prec * np.abs(x))
        err_cnt = np.sum(result == False)
        if result.all() == False:
            if err_cnt > size * prec:
                print("%s error, err_cnt: %d, all_cnt: %d, err_ratio: %f" %(testcase_name,err_cnt, size, float(err_cnt)/size))
                return
    else:
        print("%s required numpy object"%(testcase_name))
        return
    print("%s success, err_cnt: %d, all_cnt: %d, err_ratio: %f" %(testcase_name,err_cnt, size, float(err_cnt)/size))


def get_world_size() -> int:
    if not dist.is_available():
        return 1
    if not dist.is_initialized():
        return 1
    return dist.get_world_size()


def get_rank() -> int:
    if not dist.is_available():
        return 0
    if not dist.is_initialized():
        return 0
    return dist.get_rank()


def get_local_rank() -> int:
    """
    Returns:
        The rank of the current process within the local (per-machine) process group.
    """
    if not dist.is_available():
        return 0
    if not dist.is_initialized():
        return 0
    assert _LOCAL_PROCESS_GROUP is not None
    return dist.get_rank(group=_LOCAL_PROCESS_GROUP)


def get_local_size() -> int:
    """
    Returns:
        The size of the per-machine process group,
        i.e. the number of processes per machine.
    """
    if not dist.is_available():
        return 1
    if not dist.is_initialized():
        return 1
    return dist.get_world_size(group=_LOCAL_PROCESS_GROUP)


def is_main_process() -> bool:
    return get_rank() == 0


def synchronize():
    """
    Helper function to synchronize (barrier) among all processes when
    using distributed training
    """
    if not dist.is_available():
        return
    if not dist.is_initialized():
        return
    world_size = dist.get_world_size()
    if world_size == 1:
        return
    dist.barrier()


@functools.lru_cache()
def _get_global_gloo_group():
    """
    Return a process group based on gloo backend, containing all the ranks
    The result is cached.
    """
    if dist.get_backend() == "hccl":
        return dist.new_group(backend="gloo")
    else:
        return dist.group.WORLD


def _serialize_to_tensor(data, group):
    backend = dist.get_backend(group)
    assert backend in ["gloo", "hccl"]
    device = torch.device("cpu" if backend == "gloo" else "npu")

    buffer = pickle.dumps(data)
    if len(buffer) > 1024 ** 3:
        logger = logging.getLogger(__name__)
        logger.warning(
            "Rank {} trying to all-gather {:.2f} GB of data on device {}".format(
                get_rank(), len(buffer) / (1024 ** 3), device
            )
        )
    storage = torch.ByteStorage.from_buffer(buffer)
    tensor = torch.ByteTensor(storage).to(device=device)
    return tensor


def _pad_to_largest_tensor(tensor, group):
    """
    Returns:
        list[int]: size of the tensor, on each rank
        Tensor: padded tensor that has the max size
    """
    world_size = dist.get_world_size(group=group)
    assert (
        world_size >= 1
    ), "comm.gather/all_gather must be called from ranks within the given group!"
    local_size = torch.tensor([tensor.numel()], dtype=torch.int64, device=tensor.device)
    size_list = [
        torch.zeros([1], dtype=torch.int64, device=tensor.device) for _ in range(world_size)
    ]
    dist.all_gather(size_list, local_size, group=group)
    size_list = [int(size.item()) for size in size_list]

    max_size = max(size_list)

    # we pad the tensor because torch all_gather does not support
    # gathering tensors of different shapes
    if local_size != max_size:
        padding = torch.zeros((max_size - local_size,), dtype=torch.uint8, device=tensor.device)
        tensor = torch.cat((tensor, padding), dim=0)
    return size_list, tensor


def all_gather(data, group=None):
    """
    Run all_gather on arbitrary picklable data (not necessarily tensors).

    Args:
        data: any picklable object
        group: a torch process group. By default, will use a group which
            contains all ranks on gloo backend.

    Returns:
        list[data]: list of data gathered from each rank
    """
    if get_world_size() == 1:
        return [data]
    if group is None:
        group = _get_global_gloo_group()
    if dist.get_world_size(group) == 1:
        return [data]

    tensor = _serialize_to_tensor(data, group)

    size_list, tensor = _pad_to_largest_tensor(tensor, group)
    max_size = max(size_list)

    # receiving Tensor from all ranks
    tensor_list = [
        torch.empty((max_size,), dtype=torch.uint8, device=tensor.device) for _ in size_list
    ]
    dist.all_gather(tensor_list, tensor, group=group)

    data_list = []
    for size, tensor in zip(size_list, tensor_list):
        buffer = tensor.cpu().numpy().tobytes()[:size]
        data_list.append(pickle.loads(buffer))

    return data_list


def gather(data, dst=0, group=None):
    """
    Run gather on arbitrary picklable data (not necessarily tensors).

    Args:
        data: any picklable object
        dst (int): destination rank
        group: a torch process group. By default, will use a group which
            contains all ranks on gloo backend.

    Returns:
        list[data]: on dst, a list of data gathered from each rank. Otherwise,
            an empty list.
    """
    if get_world_size() == 1:
        return [data]
    if group is None:
        group = _get_global_gloo_group()
    if dist.get_world_size(group=group) == 1:
        return [data]
    rank = dist.get_rank(group=group)

    tensor = _serialize_to_tensor(data, group)
    size_list, tensor = _pad_to_largest_tensor(tensor, group)

    # receiving Tensor from all ranks
    if rank == dst:
        max_size = max(size_list)
        tensor_list = [
            torch.empty((max_size,), dtype=torch.uint8, device=tensor.device) for _ in size_list
        ]
        dist.gather(tensor, tensor_list, dst=dst, group=group)

        data_list = []
        for size, tensor in zip(size_list, tensor_list):
            buffer = tensor.cpu().numpy().tobytes()[:size]
            data_list.append(pickle.loads(buffer))
        return data_list
    else:
        dist.gather(tensor, [], dst=dst, group=group)
        return []


def shared_random_seed():
    """
    Returns:
        int: a random number that is the same across all workers.
            If workers need a shared RNG, they can use this shared seed to
            create one.

    All workers must call this function, otherwise it will deadlock.
    """
    ints = np.random.randint(2 ** 31)
    all_ints = all_gather(ints)
    return all_ints[0]


def reduce_dict(input_dict, average=True):
    """
    Reduce the values in the dictionary from all processes so that process with rank
    0 has the reduced results.

    Args:
        input_dict (dict): inputs to be reduced. All the values must be scalar CUDA Tensor.
        average (bool): whether to do average or sum

    Returns:
        a dict with the same keys as input_dict, after reduction.
    """
    world_size = get_world_size()
    if world_size < 2:
        return input_dict
    with torch.no_grad():
        names = []
        values = []
        # sort the keys so that they are consistent across processes
        for k in sorted(input_dict.keys()):
            names.append(k)
            values.append(input_dict[k])
        values = torch.stack(values, dim=0)
        dist.reduce(values, dst=0)
        if dist.get_rank() == 0 and average:
            # only main process gets accumulated, so only divide by
            # world_size in this case
            values /= world_size
        reduced_dict = {k: v for k, v in zip(names, values)}
    return reduced_dict
