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
 * @file add.h
 *
 * @brief
 *
 * @version 1.0
 *
 */

#ifndef GE_OPS_OP_PROTO_ASSIGN_H_
#define GE_OPS_OP_PROTO_ASSIGN_H_
#include "graph/operator_reg.h"
namespace ge {
REG_OP(Assign)
    .INPUT(ref, TensorType::BasicType())
    .INPUT(value,TensorType::BasicType())
    .OUTPUT(ref, TensorType::BasicType())
    .ATTR(validate_shape, Bool, true)
    .ATTR(use_locking, Bool, false)
    .OP_END_FACTORY_REG(Assign)

}

#endif //GE_OPS_OP_PROTO_ASSIGN_H_
