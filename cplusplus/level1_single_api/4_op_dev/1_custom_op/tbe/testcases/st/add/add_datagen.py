"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this file
except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

"""

import numpy as np
import sys
from dataFormat import *

def add(name, shape_x, shape_y, src_type):
    sys.stdout.write("Info: writing input for %s...\n"%name)
    shape_str = ""
    for dim in shape_x:
        shape_str += str(dim) + "_"
    feature_name = shape_str + src_type

    input_x = np.random.uniform(-2, 2, shape_x).astype(src_type)
    dumpData(input_x, name + "_input1_" + feature_name + ".data",
        fmt="binary", data_type=src_type,
        path="../data/" + name + "/" + feature_name)

    input_y = np.random.uniform(-2, 2, shape_y).astype(src_type)
    dumpData(input_y, name + "_input2_" + feature_name + ".data",
        fmt="binary", data_type=src_type,
        path="../data/" + name + "/" + feature_name)
    sys.stdout.write("Info: writing input for %s done!!!\n" % name)

    output = np.add(input_x, input_y)
    dumpData(output, name + "_output_" + feature_name + ".data",
        fmt="binary", data_type=src_type,
        path="../data/" + name + "/" + feature_name)
    sys.stdout.write("Info: writing output for %s done!!!\n"%name)

def gen_add_data(isBBIT=False):
    add("add", (1, 1), (1, 1), "float32")
    add("add", (16, 32), (16, 32), "float32")

if __name__ =="__main__":
    gen_add_data()
