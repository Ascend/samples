# -*- coding: utf-8 -*-
# this file is used for generate all type date

import numpy as np
import math
import sys
import os
import argparse
from logging import *
import logging


def getValue(obj, val):
    return obj.get(val)

npTypes = {
    "float16": np.float16,
    "float32": np.float32,
    "int8": np.int8,
    "uint8": np.uint8,
    "int32": np.int32,
    "uint32": np.uint32
}


def getNpType(typestr):
    return getValue(npTypes, typestr)
# caffe fp32
# tensorflow f32
# lianghua int8 影响权值


def errorExit(msg, *args, **kwargs):
    error(msg, *args, **kwargs)
    exit()


def convertToShape(shapeStr):
    try:
        shape = eval(shapeStr)
    except:
        errorExit("%s shape is invalid", shapeStr)
    return shape


def main(args):
    output = args.output_prefix
    datatype = getNpType(args.dtype)
    output_name = output + "_" + args.dtype + "_"
    shape = convertToShape(args.shape)
    for i in range(len(shape)):
        output_name += str(shape[i])
        if i != len(shape) - 1:
            output_name += "x"
    output_name += ".bin.in"
    data = np.random.uniform(0, 1, shape)
    rg = convertToShape(args.range)
    if len(rg) != 2:
        errorExit("range must be 2-dims")
    data = data * (rg[1] - rg[0]) + rg[0]
    data = data.astype(datatype)
    data.tofile(output_name)
    print(output_name)

if __name__ == "__main__":
    logging.basicConfig(
        format="%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s", level=logging.DEBUG)
    argsParse = argparse.ArgumentParser(
        prog=sys.argv[0], description="This script is used to generate random date bin", epilog="Enjoy it.")
    argsParse.add_argument("output_prefix", help="the output file name will be output_type_nxcxhxw.bin.in")
    argsParse.add_argument("-s", "--shape", help="input bin shape. e.g. [1,2,3,4]", required=True)
    argsParse.add_argument(
        "-r", "--range", help="output value range. only support two dimesion[low, high] e.g. [2,3]", default="[0,255]")
    argsParse.add_argument("-d", "--dtype", help="data type",
                           choices=["int8", "uint8", "float16", "float32", "int32", "uint32"], default="float32")
    args = argsParse.parse_args()
    main(args)
