/* Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use
 * this file except in compliance with the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include "tvm_bbit.hpp"
#include "register.hpp"
#include <functional>
#include <map>
#include <string>

using namespace std;

class add_Test : public BaseBbitTest{
public:
    add_Test(){
        for (const auto& item : test_calls_) {
            testcases.push_back(item.first);
        }
    };

    virtual ~add_Test() {};

    bool test(string name);
    bool test_1_1_float32();
    bool test_16_32_float32();

private:
    std::map<std::string, std::function<bool(void)>> test_calls_ = {
        {"test_1_1_float32", std::bind(&add_Test::test_1_1_float32, this)},
        {"test_16_32_float32", std::bind(&add_Test::test_16_32_float32, this)},
    };

    const std::string case_ = "add";
};

REGISTER_CLASS(add_Test)
