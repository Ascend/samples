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

test add
"""

import unittest
import os
import shutil
from impl.add import add
from run_testcase import run_testcase,get_path_val,print_func_name

testcases = {
    "op_name": "add",
    "all": {
        "test_st_add_1": ((1, 1), (1, 1), "float32", "cce_add_1_1_float32"),
        "test_st_add_2": ((16, 32), (16, 32), "float32", "cce_add_16_32_float32"),
    },
    "mini": {},
    "cloud": {},
}

bin_path_val = get_path_val(testcases)


def test_add(shape_x, shape_y, dtype_val, kernel_name_val):
    add({"shape": shape_x, "dtype": dtype_val},
        {"shape": shape_y, "dtype": dtype_val}, {},
        kernel_name=kernel_name_val)
    kernel_meta_path = "./kernel_meta/"
    lib_kernel_name = "lib" + kernel_name_val + ".so"
    if os.path.isfile(kernel_meta_path + lib_kernel_name):
        shutil.move(kernel_meta_path + lib_kernel_name,
                    bin_path_val + "/" + lib_kernel_name)
    else:
        shutil.move(kernel_meta_path + kernel_name_val + ".o",
                    bin_path_val + "/" + kernel_name_val + ".o")
        shutil.move(kernel_meta_path + kernel_name_val + ".json",
                    bin_path_val + "/" + kernel_name_val + ".json")


class Test_add_cce(unittest.TestCase):
    def tearDown(self):
        pass

    def setUp(self):
        pass

    @classmethod
    def tearDownClass(self):
        pass

    @classmethod
    def setUpClass(self):
        pass

    @print_func_name
    def test_cce_add(self):
        run_testcase(testcases, test_add)


if __name__ == "__main__":
    unittest.main()
