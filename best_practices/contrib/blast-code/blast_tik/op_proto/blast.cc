#include "blast.h"
#include <string>
#include <vector>
namespace ge {
    static bool CheckTwoInputDtypeSame(const Operator &op, const string &input_name1,
    const string &input_name2) {
        DataType input_type_x1 = op.GetInputDescByName(input_name1.c_str()).GetDataType();
        DataType input_type_x2 = op.GetInputDescByName(input_name2.c_str()).GetDataType();
        if (input_type_x1 != input_type_x2) {
            return false;
        }

        return true;
    }
IMPLEMT_COMMON_INFERFUNC(BlastInferShape)
{
    // 获取seq2的第一维大小，构建Shape
    ge::Shape seq2_shape = op.GetInputDescByName("seq2").GetShape();
    std::vector<int64_t> dim_output = seq2_shape.GetDims();
    std::vector<int64_t> dimVec;
    dimVec.push_back(dim_output[0]);
    ge::Shape outputShape = ge::Shape(dimVec);

    TensorDesc tensordesc_output = op.GetOutputDescByName("out");
    tensordesc_output.SetShape(outputShape);
    tensordesc_output.SetDataType(op.GetInputDescByName("seq1").GetDataType());
    tensordesc_output.SetFormat(op.GetInputDescByName("seq1").GetFormat());
    (void)op.UpdateOutputDesc("out", tensordesc_output);
    return GRAPH_SUCCESS;
}

IMPLEMT_VERIFIER(Blast, BlastVerify)
{
    if (!CheckTwoInputDtypeSame(op, "seq1", "seq2")) {
        return GRAPH_FAILED;
    }
    return GRAPH_SUCCESS;
}

COMMON_INFER_FUNC_REG(Blast, BlastInferShape);
VERIFY_FUNC_REG(Blast, BlastVerify);

}  // namespace ge
