#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

void preprocess(const cv::Mat& src, cv::Mat& dst)
{
    src.convertTo(dst, CV_32FC3);

    cv::cvtColor(dst, dst, cv::COLOR_BGR2RGB);
    // center crop
    int resizeWidth = 256;
    int resizeHeight = 256;
    cv::resize(dst, dst, cv::Size(resizeWidth, resizeHeight));
    int cropSize = 16;
    int inputSize = 224;
    cv::Rect roi(cropSize, cropSize, inputSize, inputSize);
    dst = dst(roi);

    float scalar = 1.0 / 255.0;
    float scalarR = 0.485;
    float scalarG = 0.456;
    float scalarB = 0.406;
    float scalarDivR = 0.229;
    float scalarDivG = 0.224;
    float scalarDivB = 0.225;
    dst = cv::dnn::blobFromImage(dst, scalar, cv::Size(), cv::Scalar(scalarR, scalarG, scalarB));
    cv::divide(dst, cv::Scalar(scalarDivR, scalarDivG, scalarDivB), dst);
}

void softmax(const cv::Mat& src, cv::Mat& dst, int axis=1)
{
    using namespace cv::dnn;

    LayerParams lp;
    Net netSoftmax;
    netSoftmax.addLayerToPrev("softmaxLayer", "Softmax", lp);
    netSoftmax.setPreferableBackend(DNN_BACKEND_OPENCV);

    netSoftmax.setInput(src);
    cv::Mat out = netSoftmax.forward();
    out.copyTo(dst);
}

int main(int argc, char** argv)
{
    using namespace cv;

    Mat image = imread("../data/dog1_1024_683.jpg"); // replace with the path to your image
    Mat input_blob;
    preprocess(image, input_blob);

    dnn::Net net = dnn::readNet("../model/image_classification_mobilenetv1_2022apr.onnx"); 

    net.setPreferableBackend(dnn::DNN_BACKEND_CANN);
    net.setPreferableTarget(dnn::DNN_TARGET_NPU);

    net.setInput(input_blob);
    Mat out = net.forward();

    Mat prob;
    softmax(out, prob, 1);

    double min_val, max_val;
    Point min_loc, max_loc;
    minMaxLoc(prob, &min_val, &max_val, &min_loc, &max_loc);
    std::cout << cv::format("cls = %d, score = %.4f\n", max_loc.x, max_val);

    return 0;
}
