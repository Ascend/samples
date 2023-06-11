/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 */
#include "add_custom_tiling.h"
#include "register/op_def_registry.h"

namespace optiling {
struct TilingCompileInfo {
    int64_t ub_size;
};

const uint32_t BLOCK_DIM = 8;
const uint32_t TILE_NUM = 8;
static ge::graphStatus TilingFunc(gert::TilingContext* context)
{
    AddCustomTilingData tiling;
    uint32_t totalLength = context->GetInputTensor(0)->GetShapeSize();
    context->SetBlockDim(BLOCK_DIM);
    tiling.set_blockDim(BLOCK_DIM);
    tiling.set_totalLength(totalLength);
    tiling.set_tileNum(TILE_NUM);
    tiling.SaveToBuffer(context->GetRawTilingData()->GetData(), context->GetRawTilingData()->GetCapacity());
    context->GetRawTilingData()->SetDataSize(tiling.GetDataSize());
    context->SetTilingKey(1);
    size_t usrSize = 256;
    size_t sysWorkspaceSize = 16777216;
    size_t *currentWorkspace = context->GetWorkspaceSizes(1);
    if (context->GetWorkspaceNum() <= 0) {
        std::cout << "GetWorkspaceNum Failed"<<std::endl;
        return ge::GRAPH_FAILED;
    }
    currentWorkspace[0] = usrSize + sysWorkspaceSize;
    return ge::GRAPH_SUCCESS;
}

ge::graphStatus TilingPrepare(gert::TilingParseContext* context)
{
    return ge::GRAPH_SUCCESS;
}

int32_t CheckOpSupport(const ge::Operator &op, ge::AscendString &result)
{
    std::string resJsonStr = "{\"ret_code\": \"0\",\"reason\": \"check_supported_stub\"}";
    result = ge::AscendString(resJsonStr.c_str());
    return 1;
}
} // namespace optiling

ge::graphStatus InferShape(gert::InferShapeContext* context)
{
    auto inputShape = context->GetInputShape(0);
    auto outputShape = context->GetOutputShape(0);
    *outputShape = *inputShape;
    return ge::GRAPH_SUCCESS;
}

ge::graphStatus InferShapeRange(gert::InferShapeRangeContext* context)
{
    auto inputShapeRange = context->GetInputShapeRange(0);
    auto outputShapeRange = context->GetOutputShapeRange(0);
    *outputShapeRange = *inputShapeRange;
    return ge::GRAPH_SUCCESS;
}

ge::graphStatus InferDataType(gert::InferDataTypeContext* context)
{
    auto inputDataType = context->GetInputDataType(0);
    context->SetOutputDataType(0, inputDataType);
    return ge::GRAPH_SUCCESS;
}

namespace ops {
class AddCustom : public OpDef {
public:
    explicit AddCustom(const char* name) : OpDef(name)
    {
        this->Input("x")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT16})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("y")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT16})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Output("z")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT16})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});

        this->SetInferShape(InferShape)
            // .SetInferShapeRange(InferShapeRange)
            .SetInferDataType(InferDataType);

        this->AICore()
            .SetTiling(optiling::TilingFunc)
            .SetTilingParse(optiling::TilingPrepare)
            .SetCheckSupport(optiling::CheckOpSupport);

        OpAICoreConfig aicConfig;
        aicConfig.AsyncFlag(true)
            .DynamicCompileStaticFlag(true)
            .DynamicFormatFlag(true)
            .DynamicRankSupportFlag(true)
            .DynamicShapeSupportFlag(true)
            .NeedCheckSupportFlag(true)
            .PrecisionReduceFlag(true)
            .RangeLimitValue("limited");

        this->AICore().AddConfig("ascend910", aicConfig);
        this->AICore().AddConfig("ascend310p", aicConfig);
        this->AICore().AddConfig("ascend910b", aicConfig);
    }
};

OP_ADD(AddCustom, optiling::TilingCompileInfo);
} // namespace ops
