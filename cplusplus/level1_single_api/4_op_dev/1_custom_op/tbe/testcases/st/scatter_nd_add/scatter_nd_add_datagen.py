import sys
import numpy as np
import tensorflow as tf
from dataFormat import *

def gen_scatter_nd_add_data(isBBIT=False):
    scatter_nd_add("scatter_nd_add", (31,2,2), (2,1), (2,2,2), src_type="float32")
    scatter_nd_add("scatter_nd_add", (17,5,5), (33,1), (33,5,5), src_type="float16")
    if isBBIT:
        scatter_nd_add("scatter_nd_add", (31,2,2), (2,1), (2,2,2), src_type="float32")
        scatter_nd_add("scatter_nd_add", (17,5,5), (33,1), (33,5,5), src_type="float16")

def scatter_nd_add(name, shape_var, shape_indices, shape_updates, src_type):
    sys.stdout.write("Info: writing input for %s...\n"%name)

    if src_type == "fp16" or src_type == "float16":
        s_type = NP.float16
    elif src_type == "fp32" or src_type == "float32":
        s_type = NP.float32
    elif src_type == "int32":
        s_type = NP.int32
    elif src_type == "int8":
        s_type = NP.int8
    elif src_type == "uint8":
        s_type = NP.uint8


    else:
        raise RuntimeError("unsupported dtype:%s "%src_type)

    shape_str = ""
    for dim in shape_updates:
        shape_str += str(dim) + "_"
    shape_name = shape_str[0:len(shape_str) - 1]
    case_name = shape_name + "_" + src_type
    path = "./../data/" + name + "/" + case_name

    shape_str_var = ""
    for dim_var in shape_var:
        shape_str_var += str(dim_var) + "_"
    shape_name_var = shape_str_var[0:len(shape_str_var) - 1]
    case_name_var = shape_name_var + "_" + src_type

    shape_str_indices = ""
    for dim_indices in shape_indices:
        shape_str_indices += str(dim_indices) + "_"
    shape_name_indices = shape_str_indices[0:len(shape_str_indices) - 1]
    case_name_indices = shape_name_indices + "_int32"

    var_min,var_max = 1,10

    if src_type == "int32":
        min, max = 0, 0
    else:
        min, max = 10, 10

    inputArr1 = NP.random.uniform(var_min, var_max, shape_var).astype(s_type)
    inputArr1_var = tf.Variable(inputArr1,dtype = s_type)
    dumpData(inputArr1, name + "_input_" + case_name_var + ".data", fmt="binary",
             data_type=src_type, path=path)
    inputArr2 = NP.random.uniform(0, 2, shape_indices).astype(NP.int32)
    dumpData(inputArr2, name + "_input_B_" + case_name_indices + ".data", fmt="binary",
             data_type="int32", path=path)

    inputArr3 = NP.random.uniform(min, max, shape_updates).astype(s_type)
    dumpData(inputArr3, name + "_input_C_" + case_name + ".data", fmt="binary",
             data_type=src_type, path=path)

    outputArr = tf.scatter_nd_add(inputArr1_var, inputArr2, inputArr3, use_locking=False)

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())
        outputArr = sess.run(outputArr)

    dumpData(outputArr, name + "_output_" + case_name_var + ".data", fmt="binary",
             data_type=src_type, path=path)

    sys.stdout.write("Info: writing output for %s done!!!\n"%name)

if __name__=="__main__":
    gen_scatter_nd_add_data(False)