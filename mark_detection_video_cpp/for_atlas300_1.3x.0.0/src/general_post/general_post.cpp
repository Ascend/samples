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

#include "general_post.h"

#include <unistd.h>
#include <algorithm>
#include <regex>
#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include "hiaiengine/log.h"
#include "tool_api.h"
#include "ascenddk/presenter/agent/presenter_types.h"

using hiai::Engine;
using namespace ascend::presenter;
using namespace std;

namespace {
// callback port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

const std::string kFaceLabelTextSuffix = "%";

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

// size of output tensor vector should be 2.
const uint32_t kOutputTensorSize = 3;
const uint32_t kOutputNumIndex = 0;


// 浮点数精度为3
const uint32_t kScorePrecision = 3;

// port number range
const int32_t kPortMinNumber = 0;
const int32_t kPortMaxNumber = 65535;

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
const static std::vector<uint32_t>  anchors = {116,90,156,198,163,326,30,61,62,45,59,119,10,13,16,30,33,23};

const static std::vector<uint32_t>  kGridSize_x= {20,40,80};
const static std::vector<uint32_t>  kGridSize_y= {11,22,44};

// channel name regular expression
const std::string kChannelNameRegularExpression = "[a-zA-Z0-9/]+";

// IP regular expression
const std::string kIpRegularExpression =
"^((25[0-5]|2[0-4]\\d|[1]{1}\\d{1}\\d{1}|[1-9]{1}\\d{1}|\\d{1})($|(?!\\.$)\\.)){4}$";
}
 // namespace

HIAI_StatusT GeneralPost::Init(
	const hiai::AIConfig& config,
	const vector<hiai::AIModelDescription>& model_desc) {
	std::stringstream ss;
	for (int index = 0; index < config.items_size(); ++index) {
		const ::hiai::AIConfigItem& item = config.items(index);
		std::string name = item.name();
		const std::string& value = item.value();
		ss << value;

		if ("Confidence" == name) {
			ss >> confidence;
		}
		else if ("PresenterIp" == name) {
			if (IsInValidIp(value)) {
				HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
					"PresenterIp=%s which configured is invalid.",
					value.c_str());
				return HIAI_ERROR;
			}
			ss >> presenter_ip;
		}
		else if ("PresenterPort" == name) {
			ss >> presenter_port;
			// validate presenter server port
			if (IsInValidPort(presenter_port)) {
				HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
					"PresenterPort=%s which configured is invalid.",
					value.c_str());
				return HIAI_ERROR;
			}
		}
		else if (name == "ChannelName") {
			// validate channel name
			if (IsInValidChannelName(value)) {
				HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
					"ChannelName=%s which configured is invalid.",
					value.c_str());
				return HIAI_ERROR;
			}
			ss >> channel_name;
		}
		ss.clear();
	}

	// ip port channel_name 
	OpenChannelParam channel_param = {
	  presenter_ip,
	  static_cast<uint16_t>(presenter_port),
	  channel_name,
	  ContentType::kVideo
	};
	Channel* chan = nullptr;
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

std::vector<BoundingBox>  GeneralPost::decodeTensor(const std::shared_ptr<EngineTrans> &result,uint ImgW, uint Imgh)
{
    std::vector<BoundingBox> binfo;  
	
    for(uint ImgIndex =0; ImgIndex < kOutputTensorSize ;ImgIndex ++)
    {
      uint gridSize_x = kGridSize_x[ImgIndex];
      uint gridSize_y = kGridSize_y[ImgIndex];

      Output out = result->inference_res[ImgIndex];

      int32_t  size= out.size / sizeof(float);

      float *resold = new (nothrow) float[size];
      error_t mem_ret =  memcpy_s(resold, out.size , out.data.get(), out.size );
      float *res = new (nothrow) float[size];
      for(int i = 0 ;i < BoxTensorLength;i++)
      {
        for(int j = 0;j < gridSize_y;j++)
        {
          for(int k = 0; k < gridSize_x; k++)
          {
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
                cf = sigmoid(cf), 3;
                
                float MaxClass =0.0f;
                uint32_t MaxClass_Loc = 0;
                for (int j = 5;j< BoxTensorLabel; j++)
                {
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

                if( ( cf * MaxClass > MaxBoxClassThresh)&&( MaxClass > MaxClassThresh ))
                {
                  float x1 = ((bx-bw/2))*ImgW;
                  float y1 = ((by-bh/2))*Imgh;
                  float x2 = ((bx+bw/2))*ImgW;
                  float y2 = ((by+bh/2))*Imgh; 
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

std::vector<BoundingBox> GeneralPost::nonMaximumSuppression(const float nmsThresh, std::vector<BoundingBox> binfo)
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

std::vector<BoundingBox> GeneralPost::nmsAllClasses(const float nmsThresh, std::vector<BoundingBox>& binfo, const uint numClasses)
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

bool GeneralPost::IsInValidIp(const std::string& ip) {
	regex re(kIpRegularExpression);
	smatch sm;
	return !regex_match(ip, sm, re);
}

bool GeneralPost::IsInValidPort(int32_t port) {
	return (port <= kPortMinNumber) || (port > kPortMaxNumber);
}

bool GeneralPost::IsInValidChannelName(
	const std::string& channel_name) {
	regex re(kChannelNameRegularExpression);
	smatch sm;
	return !regex_match(channel_name, sm, re);
}

bool GeneralPost::SendSentinel() {
  // can not discard when queue full
  HIAI_StatusT hiai_ret = HIAI_OK;
  shared_ptr<string> sentinel_msg(new (nothrow) string);
  do {
    hiai_ret = SendData(kSendDataPort, "string",
                        static_pointer_cast<void>(sentinel_msg));
    // when queue full, sleep
    if (hiai_ret == HIAI_QUEUE_FULL) {
      HIAI_ENGINE_LOG("queue full, sleep 200ms");
      usleep(kSleepInterval);
    }
  } while (hiai_ret == HIAI_QUEUE_FULL);

  // send failed
  if (hiai_ret != HIAI_OK) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call SendData failed, err_code=%d", hiai_ret);
    return false;
  }

  usleep(100000);
  return true;
}

HIAI_StatusT GeneralPost::SendImage(const ImageFrame& inference_result) {
  usleep(100000);
	PresenterErrorCode p_ret = PresentImage(presenter_channel_.get(),
		inference_result);
	// send to presenter failed
	if (p_ret != PresenterErrorCode::kNone) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
			"Send JPEG image to presenter failed, error code=%d",
			p_ret);
		return HIAI_ERROR;
	}
	return HIAI_OK;
}


HIAI_StatusT GeneralPost::PostProcess(
  const shared_ptr<EngineTrans> &result) {
  ImageFrame image_frame_para;
  image_frame_para.format = ImageFormat::kJpeg;
  image_frame_para.width = result->image_info.width;
  image_frame_para.height = result->image_info.height;
  image_frame_para.size = result->image_info.size;
  image_frame_para.data = result->image_info.data.get();

  if (result->err_msg.error) {
  	  return SendImage(image_frame_para);
  }

  vector<BoundingBox> bboxesOld,bboxesNew;
  bboxesOld = decodeTensor(result,640,352);
  bboxesNew = nmsAllClasses(nmsThresh, bboxesOld,numClasses);
  if (bboxesNew.empty()) {
    HIAI_ENGINE_LOG("There is none object detected in image .");
    return SendImage(image_frame_para);
  }
  stringstream sstream;
  vector<DetectionResult> detection_results;
  for (int i = 0; i < bboxesNew.size(); ++i) {
    DetectionResult one_result;
    one_result.lt.x = result->image_info.width * bboxesNew[i].lt_x / 640;
    one_result.lt.y = result->image_info.height * bboxesNew[i].lt_y / 352;
    one_result.rb.x = result->image_info.width * bboxesNew[i].rb_x / 640;
    one_result.rb.y = result->image_info.height * bboxesNew[i].rb_y / 352;

    printf("%d %d %d %d\n",one_result.lt.x,one_result.lt.y,one_result.rb.x,one_result.rb.y);
    sstream.str("");
    sstream << label[bboxesNew[i].attribute] << ": ";
    sstream.precision(kScorePrecision);
    sstream << 100 * bboxesNew[i].score << "%";
    one_result.result_text = sstream.str();
    detection_results.emplace_back (one_result);
  }
  image_frame_para.detection_results = detection_results;
  return SendImage(image_frame_para);
}

HIAI_IMPL_ENGINE_PROCESS("general_post", GeneralPost, INPUT_SIZE) {
HIAI_StatusT ret = HIAI_OK;

// check arg0
if (arg0 == nullptr) {
  ERROR_LOG("Failed to deal file=nothing. Reason: arg0 is empty.");
  return HIAI_ERROR;
}

// just send to callback function when finished
shared_ptr<EngineTrans> result = static_pointer_cast<EngineTrans>(arg0);
if (result->is_finished) {
  if (SendSentinel()) {
    return HIAI_OK;
  }
  ERROR_LOG("Failed to send finish data. Reason: SendData failed.");
  ERROR_LOG("Please stop this process manually.");
  return HIAI_ERROR;
}

// inference failed
if (result->err_msg.error) {
  ERROR_LOG("%s", result->err_msg.err_msg.c_str());
  return HIAI_ERROR;
}

// arrange result
  return PostProcess(result);
}