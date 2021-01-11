# -*- coding:utf-8 -*-
import unittest
from impl.scatter_nd_add import scatter_nd_add
from te import tvm
import time
import sys

sys.path.append("./llt/ops/ut/testcase_python")
from util.te_test_util import *


def common_cce(func, shape_var, shape_indices, shape_updates, dtype_var, dtype_indices, use_locking, kernel_name):
    func({"shape": shape_var, "dtype": dtype_var}, {"shape": shape_indices, "dtype": dtype_indices},
         {"shape": shape_updates, "dtype": dtype_var}, {"shape": shape_var, "dtype": dtype_var},
         use_locking=use_locking,
         kernel_name=kernel_name)


class Test_scatter_nd_add(unittest.TestCase):
    def tearDown(self):
        pass

    def setUp(self):
        pass

    @classmethod
    def tearDownClass(self):
        print("")
        print("---------------------------------------------------")

    @classmethod
    def setUpClass(self):
        print("---------------------------------------------------")
        print("[ UNITTEST START impl/scatter_nd_add.py    ]")

    def test_op_scatter_nd_add_invalid_dtype(self):
        var_valid_dtype_list = ["float16", "float32", "int32", "int8", "uint8"]
        indices_valid_dtype_list = ["int32"]
        dtype_config = create_test_dtype_config(var_valid_dtype_list)
        dtype_config_indices = create_test_dtype_config(indices_valid_dtype_list)
        for config in dtype_config:
            dtype = config[0]
            except_flag = config[1]
            if except_flag == True:
                common_cce(scatter_nd_add, (31, 2, 2), (2, 1), (2, 2, 2), dtype, "int32", use_locking=False,
                           kernel_name="error")
            else:
                try:
                    common_cce(scatter_nd_add, (31, 2, 2), (2, 1), (2, 2, 2), dtype, "int32", use_locking=False,
                               kernel_name="error")
                except RuntimeError as e:
                    pass
                else:
                    raise RuntimeError("Dtype:%s Shoud not be support !" % (dtype))

        for config in dtype_config_indices:
            dtype = config[0]
            except_flag = config[1]
            if except_flag == True:
                common_cce(scatter_nd_add, (31, 2, 2), (2, 1), (2, 2, 2), "float16", dtype, use_locking=False,
                           kernel_name="error")
            else:
                try:
                    common_cce(scatter_nd_add, (31, 2, 2), (2, 1), (2, 2, 2), "float16", dtype, use_locking=False,
                               kernel_name="error")
                except RuntimeError as e:
                    pass
                else:
                    raise RuntimeError("Indices dtype:%s Shoud not be support !" % (dtype))

        try:
            scatter_nd_add({"shape": (31, 2, 2), "dtype": "int32"}, {"shape": (2, 1), "dtype": "int32"},
                           {"shape": (2, 2, 2), "dtype": "float16"}, {"shape": (31, 2, 2), "dtype": "int32"},
                           use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

        try:
            scatter_nd_add({"shape": (31, 2, 2), "dtype": "int32"}, {"shape": (2, 1), "dtype": "int32"},
                           {"shape": (2, 2, 2), "dtype": "int32"}, {"shape": (31, 2, 2), "dtype": "float32"},
                           use_locking="False", kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

    def test_op_scatter_nd_add_invalid_shape(self):
        try:
            common_cce(scatter_nd_add, (31, 2, 2), (2, 2), (2, 2, 2), dtype_var="float16", dtype_indices="int32",
                       use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

        try:
            common_cce(scatter_nd_add, (31, 2, 2), (2, 1), (2, 2, 1), dtype_var="float32", dtype_indices="int32",
                       use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

        try:
            common_cce(scatter_nd_add, (2, 3, 4), (2, 4), (2, 2, 2), dtype_var="int32", dtype_indices="int32",
                       use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

        try:
            scatter_nd_add({"shape": (31, 2, 2), "dtype": "int32"}, {"shape": (2, 1), "dtype": "int32"},
                           {"shape": (2, 2, 2), "dtype": "int32"}, {"shape": (31, 2, 3), "dtype": "int32"},
                           use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass

        try:
            common_cce(scatter_nd_add, (31, 2, 2), (2,), (2,), dtype_var="float16", dtype_indices="int32",
                       use_locking=False, kernel_name="error")
            raise AssertionError(
                "%s.%s should throw an exception!" % (self.__class__.__name__, sys._getframe().f_code.co_name))
        except RuntimeError as e:
            pass



    def test_cce_scatter_nd_add_2_3_2_2_float32(self):
        common_cce(scatter_nd_add, (8, 7, 2, 2), (2, 3, 2), (2, 3, 2, 2), dtype_var="float32", dtype_indices="int32",
                   use_locking=False, kernel_name="cce_scatter_nd_add_2_3_2_2_fp32")

    def test_cce_scatter_nd_add_2_3_2_int32(self):
        common_cce(scatter_nd_add, (8,7,2,2), (2,3,3), (2,3,2), dtype_var="int32", dtype_indices="int32",
                   use_locking=True,  kernel_name = "cce_scatter_nd_add_2_3_2_int32")

    def test_cce_scatter_nd_add_2_3_int32(self):
        common_cce(scatter_nd_add, (8,7,2,2), (2,3,4), (2,3), dtype_var="int32", dtype_indices="int32",
                   use_locking=True,  kernel_name = "cce_scatter_nd_add_2_3_int32")

    def test_cce_scatter_nd_add_33_8_33_float32(self):
        common_cce(scatter_nd_add, (255,8,33), (33,1), (33,8,33), dtype_var="float32", dtype_indices="int32",
                   use_locking=False, kernel_name = "cce_scatter_nd_add")

    def test_cce_scatter_nd_add_2_32_32_int8(self):
        common_cce(scatter_nd_add, (4,32,32), (2,1), (2,32,32), dtype_var="int8", dtype_indices="int32",
                   use_locking=False, kernel_name = "cce_scatter_nd_add")

    def test_cce_scatter_nd_add_33_220_330_float32(self):
        common_cce(scatter_nd_add, (255,220,300), (33,1), (33,220,300), dtype_var="float32", dtype_indices="int32",
                   use_locking=False, kernel_name="cce_scatter_nd_add")

    def test_cce_scatter_nd_add_32_220_330_int8(self):
        common_cce(scatter_nd_add, (255,220,300), (32,1), (32,220,300), dtype_var="int8", dtype_indices="int32",
                   use_locking=False, kernel_name="cce_scatter_nd_add")

    def test_cce_scatter_nd_add_220_300_33_int8(self):
        common_cce(scatter_nd_add, (255, 33), (220, 300, 1), (220, 300, 33), dtype_var="int8", dtype_indices="int32",
                   use_locking=False, kernel_name="cce_scatter_nd_add")

    def test_cce_scatter_nd_add_220_300_3300_uint8(self):
        common_cce(scatter_nd_add, (255, 30000), (300, 200, 1), (300, 200, 30000), dtype_var="int8", dtype_indices="int32",
                   use_locking=False, kernel_name="cce_scatter_nd_add")

def main():
    unittest.main()
    exit(0)

if __name__ == "__main__":
    main()
