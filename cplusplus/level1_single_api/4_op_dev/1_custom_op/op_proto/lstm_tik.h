/**
 * Copyright (C)  2020. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @file lstm_tik.h
 *
 * @brief
 *
 * @version 1.0
 *
 */

#ifndef GE_OPS_OP_PROTO_LSTMTIK_H_
#define GE_OPS_OP_PROTO_LSTMTIK_H_
#include "graph/operator_reg.h"
namespace ge {
REG_OP(LSTMTik)
    .INPUT(x,
           TensorType({DT_FLOAT16}))
    .INPUT(init_h,
           TensorType({DT_FLOAT16}))
    .INPUT(init_c,
           TensorType({DT_FLOAT16}))
    .INPUT(weight,
           TensorType({DT_FLOAT16}))
    .INPUT(bias, TensorType({DT_FLOAT}))
    .OUTPUT(output_h,
            TensorType({DT_FLOAT16}))
    .OUTPUT(output_c,
            TensorType({DT_FLOAT16}))
    .OP_END_FACTORY_REG(LSTMTik)
}

#endif //GE_OPS_OP_PROTO_LSTMTIK_H_
