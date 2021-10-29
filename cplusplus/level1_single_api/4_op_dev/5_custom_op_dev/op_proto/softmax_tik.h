#ifndef GE_OP_SOFTMAXTIK_H
#define GE_OP_SOFTMAXTIK_H

#include "graph/operator_reg.h"

namespace ge {
REG_OP(SoftMaxTik)
    .INPUT(x1, TensorType({DT_FLOAT, DT_FLOAT16}))
    .INPUT(x2, TensorType({DT_FLOAT, DT_FLOAT16}))
    .OUTPUT(y, TensorType({DT_FLOAT, DT_FLOAT16}))            
    .OP_END_FACTORY_REG(SoftMaxTik)
}

#endif //GE_OP_SOFTMAXTIK_H
