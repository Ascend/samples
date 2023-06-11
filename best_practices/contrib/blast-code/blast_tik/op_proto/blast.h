/**
 * Copyright (C)  2020-2021. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @brief
 *
 * @version 1.0
 *
 */

#ifndef GE_OP_BLAST_H
#define GE_OP_BLAST_H
#include "graph/operator_reg.h"
namespace ge {

REG_OP(Blast)
    .INPUT(seq1, TensorType({DT_INT16, DT_FLOAT16}))
    .INPUT(seq2, TensorType({DT_INT16, DT_FLOAT16}))
    .INPUT(scores, TensorType({DT_INT16, DT_FLOAT16}))
    .OUTPUT(out, TensorType({DT_INT16, DT_FLOAT16}))
    .OP_END_FACTORY_REG(Blast)
}
#endif //GE_OP_BLAST_H
