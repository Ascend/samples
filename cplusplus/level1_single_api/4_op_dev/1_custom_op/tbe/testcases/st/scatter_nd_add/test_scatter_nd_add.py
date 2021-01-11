# -*- coding:utf-8 -*-
import unittest
from impl.scatter_nd_add import scatter_nd_add
import os
import shutil
import sys

sys.path.append("./llt/ops/st_all/cce_all/testcase_python")

from run_testcase import run_testcase,get_path_val,print_func_name

testcases = {
    "op_name": "scatter_nd_add",
    "all": {
        "test_scatter_nd_add_2_2_2_float32" : ((31,2,2), (2,1), (2,2,2), "float32", "cce_scatter_nd_add_2_2_2_float32"),
        "test_scatter_nd_add_33_5_5_float16" : ((17,5,5), (33,1), (33,5,5), "float16", "cce_scatter_nd_add_33_5_5_float16")
    },
    "mini": {},
    "lite": {},
    "cloud": {},
    "tiny": {},
}

bin_path_val = get_path_val(testcases)

def test_scatter_nd_add(shape_var, shape_indices, shape_updates, src_type, kernel_name_val):
    scatter_nd_add({"shape":shape_var, "dtype":src_type}, {"shape":shape_indices, "dtype":"int32"},
                   {"shape":shape_updates, "dtype":src_type}, {"shape":shape_var, "dtype":src_type},
                   use_locking=False, kernel_name = kernel_name_val)
    kernel_meta_path = "./kernel_meta/"
    lib_kernel_name = "lib" + kernel_name_val + ".so"
    if(os.path.isfile(kernel_meta_path + lib_kernel_name)):
        shutil.move(kernel_meta_path + lib_kernel_name, bin_path_val + "/"+ lib_kernel_name)
    else:
        shutil.move(kernel_meta_path + kernel_name_val + ".o", bin_path_val + "/"+ kernel_name_val + ".o")

    shutil.move(kernel_meta_path + kernel_name_val + ".json", bin_path_val + "/"+ kernel_name_val + ".json")

class Test_scatter_nd_add(unittest.TestCase):
    def tearDown(self):
        # 每个测试用例执行之后做操作
        pass

    def setUp(self):
        # 每个测试用例执行之前做操作
        pass

    @classmethod
    def tearDownClass(self):
        pass

    @classmethod
    def setUpClass(self):
        pass

    @print_func_name
    def test_scatter_nd_add(self):
        run_testcase(testcases, test_scatter_nd_add)

def main():
    unittest.main()
    exit(0)


if __name__ == "__main__":
    main()
