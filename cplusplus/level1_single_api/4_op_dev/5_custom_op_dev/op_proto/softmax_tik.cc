#include "softmax_tik.h"
#include <string>
#include <vector>

namespace ge {

IMPLEMT_VERIFIER(SoftMaxTik, SoftMaxTikVerify)
{
    std::vector<DataType> support_list;
    support_list.reserve(2);
    support_list.push_back(DT_FLOAT16);
    support_list.push_back(DT_FLOAT);
    return GRAPH_SUCCESS;
}

// Obtains the processing function of the output tensor description.
IMPLEMT_COMMON_INFERFUNC(SoftMaxTikInferShape)
{
    TensorDesc tensordesc_output = op.GetOutputDescByName("y");
    ge::TensorDesc inputTensorDescX = op.GetInputDescByName("x1");
    ge::TensorDesc inputTensorDescY = op.GetInputDescByName("x2");
    ge::Shape shapeX = inputTensorDescX.GetShape();
    ge::Shape shapeY = inputTensorDescY.GetShape();
    DataType dtype = inputTensorDescX.GetDataType();
    std::vector<int64_t> dimVector;
    dimVector.push_back(shapeX.GetDim(0));
    dimVector.push_back(shapeY.GetDim(1));
    ge::Shape outputShape(dimVector);
    tensordesc_output.SetShape(outputShape);
    tensordesc_output.SetDataType(dtype);
    (void)op.UpdateOutputDesc("y", tensordesc_output);
    return GRAPH_SUCCESS;
}

//Registered inferfunction
COMMON_INFER_FUNC_REG(SoftMaxTik, SoftMaxTikInferShape);

//Registered verify function
VERIFY_FUNC_REG(SoftMaxTik, SoftMaxTikVerify);
}
