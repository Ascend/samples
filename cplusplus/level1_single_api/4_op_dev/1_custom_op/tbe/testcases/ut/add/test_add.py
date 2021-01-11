# -*- coding:utf-8 -*-

"""
Copyright (C) 2016. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

add ut case
"""

import unittest

from impl.add import add as add


def add_cce(shape_x, shape_y, dtype, kernel_name="add"):
    add({"shape":shape_x, "dtype":dtype}, {"shape":shape_y, "dtype":dtype},
        {"shape":shape_x, "dtype":dtype}, kernel_name=kernel_name)


class Test_add_cce(unittest.TestCase):
    def setUp(self):
        #Do this before each test case is executed.
        pass

    def tearDown(self):
        #Do this after each test case is executed.
        pass

    @classmethod
    def tearDownClass(self):
        # Must use the @classmethod decorator, run once after all tests have run
        print("")
        print("---------------------------------------------------")

    @classmethod
    def setUpClass(self):
        # Must use the @classmethod decorator, run once before all tests have run
        print("---------------------------------------------------")

    def test_function_case1(self):
        add_cce((1,), (1,), "float32",
                "cce_add_1_float32")

    def test_function_case2(self):
        add_cce((1, 1), (1, 1), "float32",
                "cce_add_1_1_float32")

    def test_function_case3(self):
        add_cce((11, 33), (11, 33), "float32",
                "cce_add_11_33_float32_11_33_float32")

    def test_function_failed_case1(self):
        try:
            add_cce((10, 12), (10, 11), "float32",
                    "cce_add_10_12_float32_10_11_float32")
        except RuntimeError:
            pass

    def test_function_failed_case2(self):
        try:
            add_cce((10, 13), (10, 11, 12), "float32",
                    "cce_add_10_13_float32_10_11_13_float32")
        except RuntimeError:
            pass


if __name__ == "__main__":
    unittest.main()
    exit(0)
