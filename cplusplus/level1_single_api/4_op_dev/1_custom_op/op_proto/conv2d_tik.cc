#include "conv2d_tik.h"
#include <string>
#include <vector>
#include <algorithm>

#define CHECK_FORMAT(format)  \
{                                         \
    if(ge::FORMAT_RESERVED == format) {    \
        return false;     \
    }                     \
}

namespace ge
{

static bool GetPadConv2D(ge::Operator& op,
                         int32_t ih, int32_t iw,
                         int32_t kh, int32_t kw,
                         int32_t strh, int32_t strw,
                         int32_t dilh, int32_t dilw,
                         int32_t& padt, int32_t& padb,
                         int32_t& padl, int32_t& padr) {
    std::string padStr;
    std::vector<int32_t> padList;
    // Adaptation for tensorflow
    if (GRAPH_SUCCESS == op.GetAttr("padding", padStr)){
        if (padStr.compare("SAME") == 0){
            int32_t tails_h = ih % strh;
            int32_t tails_w = iw % strw;
            int32_t dkh = dilh*(kh - 1) + 1;
            int32_t dkw = dilw*(kw - 1) + 1;
            int32_t pad_h = \
                    std::max((tails_h > 0 ? dkh - tails_h : dkh - strh), 0);
            int32_t pad_w = \
                    std::max((tails_w > 0 ? dkw - tails_w : dkw - strw), 0);
            padList.push_back(pad_h / 2);
            padList.push_back(pad_h / 2 + pad_h % 2);
            padList.push_back(pad_w / 2);
            padList.push_back(pad_w / 2 + pad_w % 2);
        } else if (padStr.compare("VALID") == 0) {
            padList.push_back(0);
            padList.push_back(0);
            padList.push_back(0);
            padList.push_back(0);
        } else {
            return false;
        }
        op.SetAttr("pads", padList);
    }
    std::vector<int32_t> padVec;
    if (GRAPH_SUCCESS != op.GetAttr("pads", padVec)){
        return false;
    }
    auto pSize = padVec.size();
    if (pSize != 4) {
        return false;
    }
    padt = padVec[0];
    padb = padVec[1];
    padl = padVec[2];
    padr = padVec[3];
    if (padt < 0 || padb < 0 || padl < 0 || padr < 0) {
        ERROR_LOG("The value of pad should not be negative. pad = [%d, %d, %d, %d]", padt, padb, padl, padr);
        return false;
    }

    return true;
}

/*
 * Get 2D(H/W) stride and dilation params to infershape output
 *   [strides]: 4D list, format sensitive, according to first input
 *              tensor format
 *   [dilations]: 4D list, format sensitive
*/
static bool GetAttrsConv2D(ge::Operator& op, Format refer,
                           int32_t& strh, int32_t& strw,
                           int32_t& dilh, int32_t& dilw) {
    std::vector<int32_t> strideList;
    if (GRAPH_SUCCESS != op.GetAttr("strides", strideList)){
        return false;
    }
    auto sSize = strideList.size();
    if (sSize != 4) {
        return false;
    }
    std::vector<int32_t> dilationList;
    if (GRAPH_SUCCESS != op.GetAttr("dilations", dilationList)){
        return false;
    }
    auto dSize = dilationList.size();
    if (dSize != 4) {
        return false;
    }

    if (refer == FORMAT_NCHW) {
        strh = strideList[2];
        strw = strideList[3];
        dilh = dilationList[2];
        dilw = dilationList[3];
    } else if (refer == FORMAT_NHWC) {
        strh = strideList[1];
        strw = strideList[2];
        dilh = dilationList[1];
        dilw = dilationList[2];
    }
    if (strh <= 0 || strw <= 0) {
        ERROR_LOG("strh and strw should both > 0. strh = %d, strw = %d", strh, strw);
        return false;
    }
    if (dilh <= 0 || dilw <= 0) {
        ERROR_LOG("dilh and dilw should both > 0. dilh = %d, dilw = %d", dilh, dilw);
        return false;
    }

    return true;
}

/*
* Infer output shape and dtype, dtype is same to first input tensor
* Output format is set by ge parser process already
*/
IMPLEMT_INFERFUNC(Conv2DTik, Conv2DInfer) {

    auto xTensor = op.get_input_desc_x();
    auto wTensor = op.get_input_desc_filter();

    auto xShape = xTensor.GetShape().GetDims();
    auto wShape = wTensor.GetShape().GetDims();
    auto xFormat = xTensor.GetFormat();
    auto wFormat  = wTensor.GetFormat();
    CHECK_FORMAT(xFormat);
    CHECK_FORMAT(wFormat);

    int32_t in = 0;
    int32_t ic = 0;
    int32_t ih = 0;
    int32_t iw = 0;
    int32_t kn = 0;
    int32_t kc = 0;
    int32_t kh = 0;
    int32_t kw = 0;
    if (xFormat == FORMAT_NCHW) {
        in = xShape[0];
        ic = xShape[1];
        ih = xShape[2];
        iw = xShape[3];
    } else if (xFormat == FORMAT_NHWC) {
        in = xShape[0];
        ic = xShape[3];
        ih = xShape[1];
        iw = xShape[2];
    } else {
        ERROR_LOG("Feature map's format is not supported");
        return GRAPH_FAILED;
    }

    if (wFormat == FORMAT_NCHW) {
        kn = wShape[0];
        kc = wShape[1];
        kh = wShape[2];
        kw = wShape[3];
    } else if (wFormat == FORMAT_NHWC) {
        kn = wShape[0];
        kc = wShape[3];
        kh = wShape[1];
        kw = wShape[2];
    } else if (wFormat == FORMAT_HWCN) {
        kn = wShape[3];
        kc = wShape[2];
        kh = wShape[0];
        kw = wShape[1];
    } else {
        ERROR_LOG("Filter's format is not supported");
        return GRAPH_FAILED;
    }

    int64_t groups = 1;

    if (ic != kc*groups) {
        ERROR_LOG("Channel size error");
        return GRAPH_FAILED;
    }

    int32_t strh = 0;
    int32_t strw = 0;
    int32_t dilh = 0;
    int32_t dilw = 0;
    int32_t padt = 0;
    int32_t padb = 0;
    int32_t padl = 0;
    int32_t padr = 0;
    if (false == GetAttrsConv2D(op, xFormat, strh, strw, dilh, dilw)) {
        return GRAPH_FAILED;
    }
    if (false == GetPadConv2D(op, ih, iw, kh, kw, strh, strw, dilh, dilw,
                              padt, padb, padl, padr)) {
        return GRAPH_FAILED;
    }

    int64_t oh = (ih + padt + padb - dilh * (kh - 1) - 1) / strh + 1;
    int64_t ow = (iw + padl + padr - dilw * (kw - 1) - 1) / strw + 1;

    vector<int64_t> yShape;
    auto yTensor = op.get_output_desc_y();
    auto yFormat = yTensor.GetFormat();
    CHECK_FORMAT(yFormat)
    if (yFormat == FORMAT_NCHW) {
        yShape.push_back(in);
        yShape.push_back(kn);
        yShape.push_back(oh);
        yShape.push_back(ow);
    } else if (yFormat == FORMAT_NHWC) {
        yShape.push_back(in);
        yShape.push_back(oh);
        yShape.push_back(ow);
        yShape.push_back(kn);
    } else {
        ERROR_LOG("Output's format is not supported");
        return GRAPH_FAILED;
    }
    yTensor.SetShape(Shape(yShape));
    auto xDtype = xTensor.GetDataType();
    if (xDtype == ge::DT_INT8){
        yTensor.SetDataType(ge::DT_INT32);
    }else{
        yTensor.SetDataType(xDtype);
    }
    if (GRAPH_SUCCESS != op.update_output_desc_y(yTensor)) {
        return GRAPH_FAILED;
    }

    return GRAPH_SUCCESS;
}


/*
 * Verify the required 2 input tensor, optional bias ignored
 * Verify strides and dilations attrs, pads ignored
*/
IMPLEMT_VERIFIER(Conv2DTik, Conv2DVerify) {

    auto xTensor = op.get_input_desc_x();
    auto wTensor = op.get_input_desc_filter();

    auto xShape = xTensor.GetShape().GetDims();
    auto wShape = wTensor.GetShape().GetDims();

    if (xShape.size() != 4) {
        ERROR_LOG("Feature map size should be 4");
        return GRAPH_FAILED;
    }
    if (wShape.size() != 4) {
        ERROR_LOG("Filter size should be 4");
        return GRAPH_FAILED;
    }

    auto xDtype = xTensor.GetDataType();
    auto wDtype = wTensor.GetDataType();

    if(xDtype != wDtype) {
        ERROR_LOG("Inputs should have same dtype");
        return GRAPH_FAILED;
    }

    std::vector<int32_t> strideList;
    if (GRAPH_SUCCESS != op.GetAttr("strides", strideList)) {
        return GRAPH_FAILED;
    }
    std::vector<int32_t> dilationList;
    if (GRAPH_SUCCESS != op.GetAttr("dilations", dilationList)) {
        return GRAPH_FAILED;
    }

    return GRAPH_SUCCESS;
}

INFER_FUNC_REG(Conv2DTik, Conv2DInfer);
VERIFY_FUNC_REG(Conv2DTik, Conv2DVerify);

}