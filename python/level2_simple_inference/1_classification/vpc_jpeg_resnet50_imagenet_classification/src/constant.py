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
from enum import Enum
# error code
ACL_SUCCESS = 0

# data format
ACL_FORMAT_UNDEFINED = -1
ACL_FORMAT_NCHW = 0
ACL_FORMAT_NHWC = 1
ACL_FORMAT_ND = 2
ACL_FORMAT_NC1HWC0 = 3
ACL_FORMAT_FRACTAL_Z = 4


# rule for mem
class MallocType(Enum):
    ACL_MEM_MALLOC_HUGE_FIRST = 0
    ACL_MEM_MALLOC_HUGE_ONLY = 1
    ACL_MEM_MALLOC_NORMAL_ONLY = 2


# rule for memory copy
class MemcpyType(Enum):
    ACL_MEMCPY_HOST_TO_HOST = 0
    ACL_MEMCPY_HOST_TO_DEVICE = 1
    ACL_MEMCPY_DEVICE_TO_HOST = 2
    ACL_MEMCPY_DEVICE_TO_DEVICE = 3


class NpyType(Enum):
    NPY_BOOL = 0
    NPY_BYTE = 1
    NPY_UBYTE = 2
    NPY_SHORT = 3
    NPY_USHORT = 4
    NPY_INT = 5
    NPY_UINT = 6
    NPY_LONG = 7
    NPY_ULONG = 8
    NPY_LONGLONG = 9
    NPY_ULONGLONG = 10


ACL_CALLBACK_NO_BLOCK = 0
ACL_CALLBACK_BLOCK = 1


class ImageType(Enum):
    PIXEL_FORMAT_YUV_400 = 0
    PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1
    PIXEL_FORMAT_YVU_SEMIPLANAR_420 = 2
    PIXEL_FORMAT_YUV_SEMIPLANAR_422 = 3
    PIXEL_FORMAT_YVU_SEMIPLANAR_422 = 4
    PIXEL_FORMAT_YUV_SEMIPLANAR_444 = 5
    PIXEL_FORMAT_YVU_SEMIPLANAR_444 = 6
    PIXEL_FORMAT_YUYV_PACKED_422 = 7
    PIXEL_FORMAT_UYVY_PACKED_422 = 8
    PIXEL_FORMAT_YVYU_PACKED_422 = 9


# images format
IMG_EXT = ['.jpg', '.JPG', '.png', '.PNG', '.bmp',
           '.BMP', '.jpeg', '.JPEG', '.yuv']

# dvpp type
VPC_RESIZE = 0
VPC_CROP = 1
VPC_CROP_PASTE = 2
JPEG_ENC = 3
VPC_8K_RESIZE = 4
VPC_BATCH_CROP = 5
VPC_BACTH_CROP_PASTE = 6
