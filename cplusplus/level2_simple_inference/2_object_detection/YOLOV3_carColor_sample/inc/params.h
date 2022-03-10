#include "acl/acl.h"
#include <memory>
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"
#include <opencv2/core/core.hpp>

struct Rectangle {
    cv::Point lt;  // left top
    cv::Point rb;  // right bottom
};

struct CarInfo {
    ImageData cropedImgs;  // cropped image from original image
    ImageData resizedImgs;  //resized image for inference
    Rectangle rectangle;  // face rectangle
    std::string text;   //类别
    std::string carColor_result;   //颜色
    float confidence;   //颜色
};