/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File sample_process.cpp
* Description: handle acl resource
*/
#include <bits/stdint-uintn.h>
#include <iostream>
#include "acl/acl.h"
#include "model_process.h"
#include "opencv2/imgproc.hpp"
#include "utils.h"
#include "object_detect.h"
#include<string.h>
#include<sstream>
#include <string>
using namespace std;
namespace {
	uint32_t gSendNum = 0;

	//add
	string indexTableLocation = "/home/HwHiAiUser/lexicon3755.txt";
}

ObjectDetect::ObjectDetect(const char* modelPath,
                           uint32_t modelWidth,
                           uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
 modelHeight_(modelHeight), isInited_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    modelPath_ = modelPath;
    Channel* chan = nullptr;
    PresenterErrorCode openChannelret = OpenChannelByConfig(chan, "./param.conf");
    if (openChannelret != PresenterErrorCode::kNone) {
        ERROR_LOG("Open channel failed, error %d\n", (int)openChannelret);
    }
    chan_.reset(chan);

}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

Result ObjectDetect::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed\n");
        return FAILED;
    }
    INFO_LOG("acl init success\n");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed\n", deviceId_);
        return FAILED;
    }
    INFO_LOG("open device %d success\n", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed\n");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed\n");
        return FAILED;
    }
    INFO_LOG("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed\n");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath) {
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed\n");
        return FAILED;
    }

    ret = model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed\n");
        return FAILED;
    }

    ret = model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed\n");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::CreateImageInfoBuffer()
{
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed\n");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init() {
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!\n");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed\n");
        return FAILED;
    }

    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed\n");
        return FAILED;
    }

    ret = dvpp_.InitResource(stream_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed\n");
        return FAILED;
    }

    ret = CreateImageInfoBuffer();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed\n");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}


ImageResults ObjectDetect::Inference(ImageData& resizedImage, vector<string>& dict) {
	ImageResults results;
	//1.ImageData类型图像转成Mat(yuv格式)
	int img_height = resizedImage.height;
	int img_width = resizedImage.width;
	cv::Mat src(img_height * 3 / 2, img_width, CV_8UC1);
	int copy_size = img_width * img_height * 3 / 2;
	int destination_size = src.cols * src.rows * src.elemSize();
	memcpy(src.data, resizedImage.data.get(), copy_size);
	//2.src转dst_temp(yuv类型转RGB)
	cv::Mat dst_temp;
	cvtColor(src, dst_temp, cv::COLOR_YUV420sp2RGB);
	dst_temp.convertTo(dst_temp,CV_32FC3);
	//3.对dst_temp调用检测汉字算法
	std::vector<cv::Rect> rects = detect(dst_temp);
	results.num = 0;
	//对每个检测子图进行推理
	for(int j=0; j<rects.size(); j++)
	{
		//汉字预处理
      	cv::Mat roi = dst_temp(rects[j]);
      	roi = PreProcess(roi);
      	cv::resize(roi,roi,cv::Size(112,112));

		//Mat类型的图像转成ImageData类型
		uint32_t size2 = roi.total() * roi.channels();
		uint8_t *image_buf_ptr = new uint8_t[size2];
		memcpy(image_buf_ptr, roi.ptr<uint8_t>(0), size2);

		//对ImageData进行推理
		Result ret = model_.CreateInput(image_buf_ptr, size2);
		if (ret != SUCCESS) {
			ERROR_LOG("Create mode input dataset failed\n");
			return results;
		}

		ret = model_.Execute();
		if (ret != SUCCESS) {
			model_.DestroyInput();
			ERROR_LOG("Execute model inference failed\n");
			return results;
		}
		model_.DestroyInput();

		//单字推理结果
		aclmdlDataset* inferenceOutput = model_.GetModelOutputData();
        //saved result to check
        uint32_t dataSize = 0;
        float* detectData = (float *)GetInferenceOutputItem(dataSize, inferenceOutput,0);
        int result_size = dataSize/sizeof(float);
        float result[result_size];
        memcpy(result, detectData, dataSize);
        int index = 0;
        for(int m=1;m<result_size;m++)
        {
            if(result[m]>result[index])
            {
                index = m;
            }
        }
		string word = dict[index];
		word.append(":");
		word.append(std::to_string(int(result[index]*100)));
		word.append("%");
    	//添加推理结果f
		myoutput out1;
		out1.name = word;
		out1.lx = rects[j].x;
		out1.ly = rects[j].y;
		out1.rx = rects[j].x+rects[j].width;
		out1.ry = rects[j].y+rects[j].height;
    	results.output_datas.push_back(out1);

    	//总共汉字数量
    	results.num++;
	}
	return results;
}


cv::Mat ObjectDetect::PreProcess(cv::Mat src)
{
	resize(src,src,cv::Size(90,90));
	cv::Mat gray;
	cvtColor(src, gray, cv::COLOR_RGB2GRAY);
	gray.convertTo(gray,CV_8UC3);

	//Binaryzation by using THRESH method
	bitwise_not(gray, gray);
	cv::Mat thresh;
	threshold(gray, thresh, 0, 255, cv::THRESH_TOZERO | cv::THRESH_OTSU);
	bitwise_not(thresh, thresh);
	int width = src.cols;
	int height = src.rows;
	cv::Mat resize_img;

	//Resize and padding the images to 112*112
	float x = 0.0;
	if ((height > width) && (height < 224))
	{
		x = 224.0 / height;
		resize(thresh, thresh, cv::Size(int(width * x), 224));
		int padding_left = int((224 - int(width * x)) / 2);
		int padding_right = 224 - padding_left - int(width * x);
		copyMakeBorder(thresh, thresh, 0, 0, padding_left, padding_right, cv::BORDER_CONSTANT, cv::Scalar(255));

	}
	else
	{
		x = 224.0 / width;
		resize(thresh, thresh, cv::Size(224, int(height * x)));
		int padding_top = int((224 - int(height * x)) / 2);
		int padding_bottom = 224 - padding_top - int(height * x);
		copyMakeBorder(thresh, thresh, padding_top, padding_bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(255));
	}
	resize(thresh, resize_img, cv::Size(112, 112));

	/*Enhance the images if their images' meanvalue are less than110*/
	int rows = resize_img.rows;
	int cols = resize_img.cols;
	bitwise_not(resize_img, resize_img);
	float meanvalue = 0.0;
	float stdvalue = 0.0;
	int cnt = 0;

	//Calculate the meanvalue
	for (int i = 0; i < cols; i++)
	{
		for (int j = 0; j < rows; j++)
		{
			if (resize_img.at<uchar>(j, i) > 0)
			{
				stdvalue += (resize_img.at<uchar>(j, i) - meanvalue) * (resize_img.at<uchar>(j, i) - meanvalue);
			}
		}
	}
	meanvalue /= (cnt + 0.0);
	int num = 0;

	//Enhance the images
	if (meanvalue < 110)
	{
		GaussianBlur(resize_img, resize_img, cv::Size(3, 3), 0, 0);
		stdvalue /= (cnt + 0.0);
		stdvalue = sqrt(stdvalue);
		float power = log(185.0 / (185.0 + 40.0)) / log(meanvalue / (meanvalue + 2 * stdvalue));
		float alpha = 185.0 / pow(meanvalue, power);
		for (int i = 0; i < cols; i++)
		{
			for (int j = 0; j < rows; j++)
			{
				int grayvalue = resize_img.at<uchar>(j, i);
				if (grayvalue > 0)
				{
					resize_img.at<uchar>(j, i) = min(255.0, alpha * pow(grayvalue, power));
				}
			}
		}
	}
	bitwise_not(resize_img, resize_img);
	//Convert the images to RGB
	cvtColor(resize_img,resize_img,cv::COLOR_GRAY2BGR);
	return resize_img;
}


std::vector<cv::Rect> ObjectDetect::detect(cv::Mat img)
{
	img.convertTo(img,CV_8UC3);
	std::vector<cv::Rect> result_rects;
	float benchmark_len = img.rows;
	float bench_area = img.rows*img.cols;

	// extract red area contours from image
	cv::Mat hsv;
	cv::cvtColor(img.clone(), hsv, cv::COLOR_BGR2HSV);
	cv::Mat mask;
	cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask);
	cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
	cv::Mat dilate1;
	cv::dilate(mask, dilate1, element1);
	std::vector<cv::Rect> rects;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	std::vector <cv::Point> center_point;
	cv::findContours(dilate1, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0)
	{
		return result_rects;
	}

	// get the Horizontal outer rectangular boxs of this contours
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect rect = cv::boundingRect(contours[i]);
		rects.push_back(rect);
	}

	// Calculate the center distances of all rectangular boxes
	// set a threshold, If the relative distance is less than the threshold, it is considered to belong to the same region
	std::vector<std::vector<int> > dist_list(rects.size());
	for (int i = 0; i < rects.size(); i++)
	{
		dist_list[i].push_back(i);
		for (int j = i + 1; j < rects.size(); j++)
		{
			float two_dist = pow((rects[i].x + 0.5*rects[i].width) - (rects[j].x + 0.5*rects[j].width), 2)
				+ pow((rects[i].y + 0.5*rects[i].height) - (rects[j].y + 0.5*rects[j].height), 2);
			float w_scale = two_dist / benchmark_len;
			if (w_scale < 1.5)
			{
				dist_list[i].push_back(j);
			}
		}
	}

	// use Union-Find method to Merged rectangle that belong to the same region
	std::vector <int> dist_result(dist_list.size(), -1);
	std::vector<int> dist;
	for (int m = 0; m < dist_list.size(); m++)
	{
		dist = dist_list[m];
		for (int j = 0; j < dist.size(); j++)
		{
			if (dist_result[m] == -1)
			{
				dist_result[m] = dist[j];
			}
			else
			{
				dist_result[dist[j]] = dist_result[m];
			}
		}
	}

	std::vector<std::vector<int> > un_list(dist_result.size());
	std::vector<int> un_dist;
	std::vector<cv::Rect> inte_area;

	for (int i = 0; i < dist_result.size(); i++)
	{
		un_list[dist_result[i]].push_back(i);
	}

	// Get the text area outer rectangle box
	for (int m = 0; m < un_list.size(); m++)
	{
		if (un_list[m].empty())
		{
			continue;
		}
		un_dist = un_list[m];
		int xmin = rects[un_dist[0]].x;
		int ymin = rects[un_dist[0]].y;
		int xmax = rects[un_dist[0]].x + rects[un_dist[0]].width;
		int ymax = rects[un_dist[0]].y + rects[un_dist[0]].height;
		for (int j = 1; j < un_dist.size(); j++)
		{
			if (xmin > rects[un_dist[j]].x)
			{
				xmin = rects[un_dist[j]].x;
			}
			if (ymin > rects[un_dist[j]].y)
			{
				ymin = rects[un_dist[j]].y;
			}
			if (xmax < rects[un_dist[j]].x + rects[un_dist[j]].width)
			{
				xmax = rects[un_dist[j]].x + rects[un_dist[j]].width;
			}
			if (ymax < rects[un_dist[j]].y + rects[un_dist[j]].height)
			{
				ymax = rects[un_dist[j]].y + rects[un_dist[j]].height;
			}
		}

		// Calculate relative area
		// set a threshold, If the relative area is larger than the threshold, add it, else ignored
		double area = (xmax - xmin)*(ymax - ymin) / bench_area;
		if (area>0.0015 && area<0.03)
		{
			cv::Rect roi = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
			result_rects.push_back(roi);
		}
	}
	return result_rects;
}


Result ObjectDetect::Postprocess(ImageData& image, ImageResults& modelOutput) {

	uint32_t num = modelOutput.num;
	vector<DetectionResult> detectResults;
	for(uint32_t i=0;i<num;i++)
	{
		myoutput out2 = modelOutput.output_datas[i];
		DetectionResult one_result;
		Point point_lt, point_rb;
		point_lt.x = out2.lx;
		point_lt.y = out2.ly;
		point_rb.x = out2.rx;
		point_rb.y = out2.ry;


		one_result.lt = point_lt;
		one_result.rb = point_rb;
		//get corresponding Chinese character from the index table
		one_result.result_text=out2.name;
		detectResults.push_back(one_result);


	}
	ImageData jpgImage;

	//Result ret = dvpp_.CvtYuv420spToJpeg(jpgImage, image);
	//------------------------------后加--------------------------------------
	//1.ImageData类型图像转成Mat(yuv格式)
	int img_height = image.height;
	int img_width = image.width;
	cv::Mat jpegSrc(img_height * 3 / 2, img_width, CV_8UC1);
	int copy_size = img_width * img_height * 3 / 2;
	int destination_size = jpegSrc.cols * jpegSrc.rows * jpegSrc.elemSize();
	memcpy(jpegSrc.data, image.data.get(), copy_size);
	//2.src转dst_temp(yuv类型转RGB)
	cv::Mat jpegDst;
	cvtColor(jpegSrc, jpegDst, cv::COLOR_YUV420sp2RGB);
	Result ret = SendImage(chan_.get(), jpegDst, detectResults);
	return ret;

}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, idx);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer from model "
                  "inference output failed\n", idx);
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer address "
                  "from model inference output failed\n", idx);
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The %d   th dataset buffer size of "
                  "model inference output is 0\n", idx);
        return nullptr;
    }

    void* data = nullptr;
    data = dataBufferDev;

    itemDataSize = bufferSize;
    return data;
}

void ObjectDetect::DestroyResource()
{
    aclError ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed\n");
    }
    INFO_LOG("end to reset device is %d\n", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed\n");
    }
    INFO_LOG("end to finalize acl");
    aclrtFree(imageInfoBuf_);
}

//add
void ObjectDetect::EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg)
{
    vector<int> param = vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}

Result ObjectDetect::SendImage(Channel* channel,cv::Mat& image,vector<DetectionResult>& detRes) {

    //add
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, image);

    ImageFrame frame;
    frame.format = ImageFormat::kJpeg;
    frame.width = image.cols;
    frame.height = image.rows;
    frame.size = encodeImg.size();
    frame.data = reinterpret_cast<uint8_t*>(encodeImg.data());
    frame.detection_results = detRes;

    PresenterErrorCode ret = PresentImage(channel, frame);
    // send to presenter failed
    if (ret != PresenterErrorCode::kNone) {
        ERROR_LOG("Send JPEG image to presenter failed, error %d\n", (int)ret);
        return FAILED;
    }

    INFO_LOG("Send JPEG image to presenter success, ret %d,num =%d \n", (int)ret,gSendNum++);
    return SUCCESS;
}