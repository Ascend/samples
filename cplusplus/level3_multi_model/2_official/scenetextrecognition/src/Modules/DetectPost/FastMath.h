/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FASTMATH_H
#define FASTMATH_H

#include <cmath>
#include <memory>

/*
Utilize quantization and look up table to accelate exp operation
*/
class FastMath {
public:
    FastMath() {}
    inline void init()
    {
        for (auto i = 0; i < MASK_LEN; i++) {
            negCoef_[0][i] = std::exp(-float(i) / QUANT_VALUE);
            negCoef_[1][i] = std::exp(-float(i) * MASK_LEN / QUANT_VALUE);
            posCoef_[0][i] = std::exp(float(i) / QUANT_VALUE);
            posCoef_[1][i] = std::exp(float(i) * MASK_LEN / QUANT_VALUE);
        }
    }

    ~FastMath() {}
    inline float FExp(const float x) const
    {
        int quantX = std::max(std::min(x, float(QUANT_BOUND)), -float(QUANT_BOUND)) * QUANT_VALUE;
        float expx = 0;
        if (quantX & 0x80000000) {
            expx = negCoef_[0][((~quantX + 0x00000001)) & MASK_VALUE] * \
                   negCoef_[1][((~quantX + 0x00000001) >> MASK_BITS) & MASK_VALUE];
        } else {
            expx = posCoef_[0][(quantX) & MASK_VALUE] * posCoef_[1][(quantX >> MASK_BITS) & MASK_VALUE];
        }
        return expx;
    }
    inline float Sigmoid(float x) const
    {
        return 1.0f / (1.0f + FExp(-x));
    }

private:
    static const int MASK_BITS = 12; // 常量定义说明
    static const int MASK_LEN = (1 << MASK_BITS);
    static const int MASK_VALUE = MASK_LEN - 1;
    static const int QUANT_BITS = 16;
    static const int QUANT_VALUE = (1 << QUANT_BITS);
    static const int QUANT_BOUND = (1 << (2 * MASK_BITS - QUANT_BITS)) - 1;
    float negCoef_[2][MASK_LEN] = {};
    float posCoef_[2][MASK_LEN] = {};
};

namespace fastmath {
    static FastMath fastMath;
    inline float exp(const float x)
    {
        return fastMath.FExp(x);
    }
    inline float sigmoid(float x)
    {
        return fastMath.Sigmoid(x);
    }
}

#endif