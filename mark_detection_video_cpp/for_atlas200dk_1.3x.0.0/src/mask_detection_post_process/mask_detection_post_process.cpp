/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include "mask_detection_post_process.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <regex>
#include <iostream>
#include <iomanip>
#include "hiaiengine/log.h"

using hiai::Engine;
using namespace ascend::presenter;
using namespace std::__cxx11;

// register data type
HIAI_REGISTER_DATA_TYPE("EngineTransT", EngineTransT);
HIAI_REGISTER_DATA_TYPE("OutputT", OutputT);
HIAI_REGISTER_DATA_TYPE("ScaleInfoT", ScaleInfoT);
HIAI_REGISTER_DATA_TYPE("NewImageParaT", NewImageParaT);
HIAI_REGISTER_DATA_TYPE("BatchImageParaWithScaleT", BatchImageParaWithScaleT);
//HIAI_REGISTER_DATA_TYPE("BoundingBox", BoundingBox);
using namespace std;
// constants
namespace {
//// parameters for drawing box and label begin////

// face label text prefix
//const std::string kFaceLabelTextPrefix = "Face:";
const std::string kFaceLabelTextSuffix = "%";
const static std::vector<string> label = {"background","face", "person", "mask"};
//const static std::vector<string> label = {"background","person", "face", "mask"};
//// parameters for drawing box and label end////

// port number range
const int32_t kPortMinNumber = 0;
const int32_t kPortMaxNumber = 65535;

// confidence range
const float kConfidenceMin = 0.0;
const float kConfidenceMax = 1.0;

// face detection function return value
const int32_t kFdFunSuccess = 0;
const int32_t kFdFunFailed = -1;

const uint32_t kOutputTensorSize = 3;
const uint32_t kOutputNumIndex = 0;
const uint32_t kOutputTesnorIndex = 1;

const uint BoxTensorLabel  = 8;

const uint numBBoxes = 3;
const uint  BoxTensorLength = (BoxTensorLabel * numBBoxes);
const float nmsThresh = 0.45;
const float MaxBoxClassThresh = 0.3;
const float MaxClassThresh = 0.9;
const uint numClasses = 3;

const int32_t kAttributeIndex = 1;
// percent
const int32_t kScorePercent = 100;

// IP regular expression
const std::string kIpRegularExpression =
    "^((25[0-5]|2[0-4]\\d|[1]{1}\\d{1}\\d{1}|[1-9]{1}\\d{1}|\\d{1})($|(?!\\.$)\\.)){4}$";

const static std::vector<uint32_t>  anchors = {116,90,156,198,163,326,30,61,62,45,59,119,10,13,16,30,33,23};

const static std::vector<uint32_t>  kGridSize_x= {20,40,80};
const static std::vector<uint32_t>  kGridSize_y= {11,22,44};

// channel name regular expression
const std::string kChannelNameRegularExpression = "[a-zA-Z0-9/]+";
}

FaceDetectionPostProcess::FaceDetectionPostProcess() {
  fd_post_process_config_ = nullptr;
  presenter_channel_ = nullptr;
}

/*float Round(float fValue, int bits)
{
    float nbitsValue;
    std::stringstream sstream;

    sstream << fixed << setprecision(bits) << fValue;
    sstream >> nbitsValue;

    return nbitsValue;

}*/

HIAI_StatusT FaceDetectionPostProcess::Init(
    const hiai::AIConfig& config,
    const std::vector<hiai::AIModelDescription>& model_desc) {
  HIAI_ENGINE_LOG("Begin initialize!");

  // get configurations
  if (fd_post_process_config_ == nullptr) {
    fd_post_process_config_ = std::make_shared<FaceDetectionPostConfig>();
  }

  // get parameters from graph.config
  for (int index = 0; index < config.items_size(); index++) {
    const ::hiai::AIConfigItem& item = config.items(index);
    const std::string& name = item.name();
    const std::string& value = item.value();
    std::stringstream ss;
    ss << value;
    if (name == "Confidence") {
      ss >> (*fd_post_process_config_).confidence;
      // validate confidence
      if (IsInvalidConfidence(fd_post_process_config_->confidence)) {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
                        "Confidence=%s which configured is invalid.",
                        value.c_str());
        return HIAI_ERROR;
      }
    } else if (name == "PresenterIp") {
      // validate presenter server IP
      if (IsInValidIp(value)) {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
                        "PresenterIp=%s which configured is invalid.",
                        value.c_str());
        return HIAI_ERROR;
      }
      ss >> (*fd_post_process_config_).presenter_ip;
    } else if (name == "PresenterPort") {
      ss >> (*fd_post_process_config_).presenter_port;
      // validate presenter server port
      if (IsInValidPort(fd_post_process_config_->presenter_port)) {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
                        "PresenterPort=%s which configured is invalid.",
                        value.c_str());
        return HIAI_ERROR;
      }
    } else if (name == "ChannelName") {
      // validate channel name
      if (IsInValidChannelName(value)) {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
                        "ChannelName=%s which configured is invalid.",
                        value.c_str());
        return HIAI_ERROR;
      }
      ss >> (*fd_post_process_config_).channel_name;
    }
    // else : nothing need to do
  }

  // call presenter agent, create connection to presenter server
  uint16_t u_port = static_cast<uint16_t>(fd_post_process_config_
      ->presenter_port);
  OpenChannelParam channel_param = { fd_post_process_config_->presenter_ip,
      u_port, fd_post_process_config_->channel_name, ContentType::kVideo };
  Channel *chan = nullptr;
  PresenterErrorCode err_code = OpenChannel(chan, channel_param);
  // open channel failed
  if (err_code != PresenterErrorCode::kNone) {
    HIAI_ENGINE_LOG(HIAI_GRAPH_INIT_FAILED,
                    "Open presenter channel failed, error code=%d", err_code);
    return HIAI_ERROR;
  }

  presenter_channel_.reset(chan);
  HIAI_ENGINE_LOG(HIAI_DEBUG_INFO, "End initialize!");
  return HIAI_OK;
}

std::vector<BoundingBox>  FaceDetectionPostProcess::decodeTensor(const std::shared_ptr<EngineTransT> &result,uint ImgW, uint Imgh)
{
    std::vector<BoundingBox> binfo;  

	  //printf("kOutputTensorSize=%d\n",kOutputTensorSize);
    /*if(kOutputTensorSize >3)
    {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INIT_FAILED,
                            "kOutputTensorSize   does not match.");
        //ERROR_LOG("kOutputTensorSize   does not match.");
        return binfo;
    }*/
	
    for(uint ImgIndex =0; ImgIndex < kOutputTensorSize ;ImgIndex ++)
    {
      uint gridSize_x = kGridSize_x[ImgIndex];
      uint gridSize_y = kGridSize_y[ImgIndex];

      //Output out = result->output_datas[ImgIndex];


      OutputT out = result->output_datas[ImgIndex];  //output_datas
      int32_t  size= out.size / sizeof(float);
      //printf("index:%d out_size: %d \n",ImgIndex,size);

      float *resold = new (nothrow) float[size];
      error_t mem_ret =  memcpy_s(resold, out.size , out.data.get(), out.size );
      float *res = new (nothrow) float[size];
      for(int i = 0 ;i < BoxTensorLength;i++)
      {
        for(int j = 0;j < gridSize_y;j++)
        {
          for(int k = 0; k < gridSize_x; k++)
          {
            //if ((((k * gridSize_x +j) * BoxTensorLength + i )>= size) ||(((i*gridSize_x+j)*gridSize_y +k)>=size) )
            /*if ((((k * gridSize_y +j) * BoxTensorLength + i )>= size) ||(((i*gridSize_y+j)*gridSize_x +k)>=size) )
            {
                //ERROR_LOG("kOutputTensorSize   does not match.");
                 //printf("kOutputTensorSize   does not match.d \n");
                return binfo;
            }*/

            //res[(k * gridSize_x +j) * BoxTensorLength + i] = resold[(i*gridSize_x+j)*gridSize_y +k];
            res[(k * gridSize_y +j) * BoxTensorLength + i] = resold[(i*gridSize_y+j)*gridSize_x +k];
          }
        }
      }

      int count =1;         
      for ( uint cx = 0; cx < gridSize_x; cx++)
      {  
        for(uint cy = 0; cy < gridSize_y; cy++)
        {   
            float MaxBoxProb = 0;            
            float bx = 0;
            float by = 0;
            float bw = 0;
            float bh = 0;
            float tx ;
            float ty ;
            float tw ;
            float th;
            float cf;
            
                        
            for (uint i = 0; i  < numBBoxes; ++i)
            {   
                const int bbindex = BoxTensorLength*(cx * gridSize_y + cy);

                tx =  res[bbindex+i * BoxTensorLabel + 0];
                ty =  res[bbindex+i * BoxTensorLabel + 1];
                tw =  res[bbindex+i * BoxTensorLabel + 2];
                th =  res[bbindex+i * BoxTensorLabel + 3];
                cf =  res[bbindex+i * BoxTensorLabel + 4];
                //cf = Round(sigmoid(cf), 3);
                cf = sigmoid(cf), 3;
                
                float MaxClass =0.0f;
                uint32_t MaxClass_Loc = 0;
                for (int j = 5;j< BoxTensorLabel; j++)
                {
                    //float class_prob =  Round(sigmoid(res[bbindex+ (i * BoxTensorLabel + j)]), 3);
                    float class_prob =  sigmoid(res[bbindex+ (i * BoxTensorLabel + j)]);
                    if(MaxClass < class_prob)
                    {
                      MaxClass = class_prob;
                      MaxClass_Loc = j - 5;
                    }
                }
                bx = (sigmoid(tx)+cx)/(gridSize_x);
                by = (sigmoid(ty)+cy)/(gridSize_y);
                bw = anchors[i*2+ ImgIndex*6] *exp(tw)/ImgW;
                bh = anchors[i*2+1+ImgIndex*6] *exp(th)/Imgh;                
              
                /*count++;
                if(cf* MaxClass >MaxBoxProb)
                {
                  MaxBoxProb = cf* MaxClass;
                } */               
                
                if( ( cf * MaxClass > MaxBoxClassThresh)&&( MaxClass > MaxClassThresh ))
                {
                  uint32_t x1 = ((bx-bw/2))*ImgW;
                  uint32_t y1 = ((by-bh/2))*Imgh;
                  uint32_t x2 = ((bx+bw/2))*ImgW;
                  uint32_t y2 = ((by+bh/2))*Imgh; 
                  binfo.push_back({x1, y1,x2, y2, MaxClass_Loc, MaxClass});
                }

            }
          }
      }    
  	  delete[] res;
  	  delete[] resold;
    }
    
    return binfo;
}

std::vector<BoundingBox> FaceDetectionPostProcess::nonMaximumSuppression(const float nmsThresh, std::vector<BoundingBox> binfo)
{
    auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
        if (x1min > x2min)
        {
            std::swap(x1min, x2min);
            std::swap(x1max, x2max);
        }
        return x1max < x2min ? 0 : std::min(x1max, x2max) - x2min;
    };
    auto computeIoU = [&overlap1D](BoundingBox& bbox1, BoundingBox& bbox2) -> float {
        float overlapX = overlap1D(bbox1.lt_x, bbox1.rb_x, bbox2.lt_x, bbox2.rb_x);
        float overlapY = overlap1D(bbox1.lt_y, bbox1.rb_y, bbox2.lt_y, bbox2.rb_y);
        float area1 = (bbox1.rb_x - bbox1.lt_x) * (bbox1.rb_y - bbox1.lt_y);
        float area2 = (bbox2.rb_x - bbox2.lt_x) * (bbox2.rb_y - bbox2.lt_y);
        float overlap2D = overlapX * overlapY;
        float u = area1 + area2 - overlap2D;
        return u == 0 ? 0 : overlap2D / u;
    };

    std::stable_sort(binfo.begin(), binfo.end(),
                     [](const BoundingBox& b1, const BoundingBox& b2) { return b1.score > b2.score; });
    std::vector<BoundingBox> out;
    for (auto& i : binfo)
    {
        bool keep = true;
        for (auto& j : out)
        {
            if (keep)
            {
                float overlap = computeIoU(i, j);
                keep = overlap <= nmsThresh;
            }
            else
                break;
        }
        if (keep) out.push_back(i);
    }
    return out;
}

std::vector<BoundingBox> FaceDetectionPostProcess::nmsAllClasses(const float nmsThresh, std::vector<BoundingBox>& binfo, const uint numClasses)
{
    std::vector<BoundingBox> result;
    std::vector<std::vector<BoundingBox>> splitBoxes(numClasses);
    for (auto& box : binfo)
    {
        splitBoxes.at(box.attribute).push_back(box);
    }

    for (auto& boxes : splitBoxes)
    {
        boxes = nonMaximumSuppression(nmsThresh, boxes);
        result.insert(result.end(), boxes.begin(), boxes.end());
    }

    return result;
}

bool FaceDetectionPostProcess::IsInValidIp(const std::string &ip) {
  regex re(kIpRegularExpression);
  smatch sm;
  return !regex_match(ip, sm, re);
}

bool FaceDetectionPostProcess::IsInValidPort(int32_t port) {
  return (port <= kPortMinNumber) || (port > kPortMaxNumber);
}

bool FaceDetectionPostProcess::IsInValidChannelName(
    const std::string &channel_name) {
  regex re(kChannelNameRegularExpression);
  smatch sm;
  return !regex_match(channel_name, sm, re);
}

bool FaceDetectionPostProcess::IsInvalidConfidence(float confidence) {
  return (confidence <= kConfidenceMin) || (confidence > kConfidenceMax);
}

bool FaceDetectionPostProcess::IsInvalidResults( float score,
                                                const Point &point_lt,
                                                const Point &point_rb) {

  // confidence check
  if ((score < fd_post_process_config_->confidence)
      || IsInvalidConfidence(score)) {
    return true;
  }

  // rectangle position is a point or not: lt == rb
  if ((point_lt.x == point_rb.x) && (point_lt.y == point_rb.y)) {
    return true;
  }
  return false;
}

int32_t FaceDetectionPostProcess::SendImage(uint32_t height, uint32_t width,
                                            uint32_t size, u_int8_t *data, std::vector<DetectionResult>& detection_results) {
  int32_t status = kFdFunSuccess;
  // parameter
  ImageFrame image_frame_para;
  image_frame_para.format = ImageFormat::kJpeg;
  image_frame_para.width = width;
  image_frame_para.height = height;
  image_frame_para.size = size;
  image_frame_para.data = data;
  image_frame_para.detection_results = detection_results;

  PresenterErrorCode p_ret = PresentImage(presenter_channel_.get(),
                                            image_frame_para);
  // send to presenter failed
  if (p_ret != PresenterErrorCode::kNone) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "Send JPEG image to presenter failed, error code=%d",
                      p_ret);
    status = kFdFunFailed;
  }

  return status;
}

HIAI_StatusT FaceDetectionPostProcess::HandleOriginalImage(
    const std::shared_ptr<EngineTransT> &inference_res) {
  HIAI_StatusT status = HIAI_OK;
  std::vector<NewImageParaT> img_vec = inference_res->imgs;
  // dealing every original image
  for (uint32_t ind = 0; ind < inference_res->b_info.batch_size; ind++) {
    uint32_t width = img_vec[ind].img.width;
    uint32_t height = img_vec[ind].img.height;
    uint32_t size = img_vec[ind].img.size;

    // call SendImage
    // 1. call DVPP to change YUV420SP image to JPEG
    // 2. send image to presenter
    vector<DetectionResult> detection_results;
    int32_t ret = SendImage(height, width, size, img_vec[ind].img.data.get(), detection_results);
    if (ret == kFdFunFailed) {
      status = HIAI_ERROR;
      continue;
    }
  }
  return status;
}

HIAI_StatusT FaceDetectionPostProcess::HandleResults(
    const std::shared_ptr<EngineTransT> &inference_res) {
  HIAI_StatusT status = HIAI_OK;
  std::vector<NewImageParaT> img_vec = inference_res->imgs;
  std::vector<OutputT> output_data_vec = inference_res->output_datas;
  // dealing every image

  vector<BoundingBox> bboxesOld,bboxesNew;
  bboxesOld = decodeTensor(inference_res,640,352);
  //printf("boxes size is: %d",bboxesOld.size());
  bboxesNew = nmsAllClasses(nmsThresh, bboxesOld,numClasses);


  uint32_t width = img_vec[0].img.width;
  uint32_t height = img_vec[0].img.height;
  uint32_t img_size = img_vec[0].img.size;

  // every inference result needs 8 float
  // loop the result for every inference result
  std::vector<DetectionResult> detection_results;
  //float *ptr = result;
  for (int32_t k = 0; k < bboxesNew.size(); k ++) {

    float score = bboxesNew[k].score;
    std::string currentlabel = label[1+bboxesNew[k].attribute]; 
    //Detection result
    DetectionResult one_result;
    // left top
    Point point_lt, point_rb;
    point_lt.x = bboxesNew[k].lt_x * width/640;
    point_lt.y = bboxesNew[k].lt_y * height/352;
    // right bottom
    point_rb.x = bboxesNew[k].rb_x * width/640;
    point_rb.y = bboxesNew[k].rb_y * height/352;

    one_result.lt = point_lt;
    one_result.rb = point_rb;

    // check results is valid
    if (IsInvalidResults(score, point_lt, point_rb)) {
      continue;
    }
    /*HIAI_ENGINE_LOG(HIAI_DEBUG_INFO,
                    "score=%f, lt.x=%d, lt.y=%d, rb.x=%d, rb.y=%d", score,
                    point_lt.x, point_lt.y, point_rb.x, point_rb.y);*/
    int32_t score_percent =  score * kScorePercent;
    one_result.result_text.append(currentlabel);
    one_result.result_text.append(to_string(score_percent));
    one_result.result_text.append(kFaceLabelTextSuffix);

    // push back
    detection_results.emplace_back(one_result);
  }

  int32_t ret;
  ret = SendImage(height, width, img_size, img_vec[0].img.data.get(), detection_results);

  // check send result
  if (ret == kFdFunFailed) {
    status = HIAI_ERROR;
  }
  return status;
}

HIAI_IMPL_ENGINE_PROCESS("mask_detection_post_process",
    FaceDetectionPostProcess, INPUT_SIZE) {
  // check arg0 is null or not
  if (arg0 == nullptr) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Failed to process invalid message.");
    return HIAI_ERROR;
  }

  // check original image is empty or not
  std::shared_ptr<EngineTransT> inference_res = std::static_pointer_cast<
      EngineTransT>(arg0);
  if (inference_res->imgs.empty()) {
    HIAI_ENGINE_LOG(
        HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
        "Failed to process invalid message, original image is null.");
    return HIAI_ERROR;
  }

  // inference failed, dealing original images
  if (!inference_res->status) {
    HIAI_ENGINE_LOG(HIAI_OK, inference_res->msg.c_str());
    HIAI_ENGINE_LOG(HIAI_OK, "will handle original image.");
    return HandleOriginalImage(inference_res);
  }

  // inference success, dealing inference results
  return HandleResults(inference_res);
}
