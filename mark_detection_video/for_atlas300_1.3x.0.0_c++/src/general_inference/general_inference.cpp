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

#include "general_inference.h"

#include <vector>
#include <sstream>
#include <unistd.h>
#include "hiaiengine/log.h"
#include "ascenddk/ascend_ezdvpp/dvpp_process.h"
#include "tool_api.h"

using hiai::Engine;
using hiai::ImageData;
using namespace std;
using namespace ascend::utils;

namespace {
const int kProcSuccess = 0;
// model_path parameter key in graph.config
const string kModelPathParamKey = "model_path";

// output port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

// model process timeout
const uint32_t kAiModelProcessTimeout = 0;

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

}


GeneralInference::GeneralInference() {
  ai_model_manager_ = nullptr;
}

HIAI_StatusT GeneralInference::Init(
    const hiai::AIConfig& config,
    const vector<hiai::AIModelDescription>& model_desc) {
  HIAI_ENGINE_LOG("Start initialize!");

  // initialize aiModelManager
  if (ai_model_manager_ == nullptr) {
    MAKE_SHARED_NO_THROW(ai_model_manager_, hiai::AIModelManager);
    if (ai_model_manager_ == nullptr) {
      ERROR_LOG("Failed to initialize AIModelManager.");
      return HIAI_ERROR;
    }
  }

  // get parameters from graph.config
  // set model path to AI model description
  hiai::AIModelDescription fd_model_desc;

  std::stringstream ss;
  for (int index = 0; index < config.items_size(); index++) {
    const ::hiai::AIConfigItem& item = config.items(index);
    ss << item.value();
    if (item.name() == kModelPathParamKey) {
        ss >> model_path;
        INFO_LOG("model path %s", model_path.c_str());
        fd_model_desc.set_path(model_path.c_str());
    } else if (item.name() == "resize_width") {
        ss >> resize_width;
    } else if (item.name() == "resize_height") {
        ss >> resize_height;
    }
    ss.clear();
  }

  // initialize model manager
  vector<hiai::AIModelDescription> model_desc_vec;
  model_desc_vec.push_back(fd_model_desc);
  hiai::AIStatus ret = ai_model_manager_->Init(config, model_desc_vec);
  // initialize AI model manager failed
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE, "initialize AI model failed");
    ERROR_LOG("Failed to initialize AI model.");
    return HIAI_ERROR;
  }

  HIAI_ENGINE_LOG("End initialize!");
  return HIAI_OK;
}

bool GeneralInference::ConvertImage(const shared_ptr<EngineTrans>& image_handle,
	                                        ImageData<u_int8_t>& jpeg_image) {
	uint32_t width = image_handle->image_info.width;
	uint32_t height = image_handle->image_info.height;
	uint32_t img_size = image_handle->image_info.size;

	// parameter
	ascend::utils::DvppToJpgPara dvpp_to_jpeg_para;
	dvpp_to_jpeg_para.format = JPGENC_FORMAT_NV12;
	dvpp_to_jpeg_para.level = 100;
	dvpp_to_jpeg_para.resolution.height = height;
	dvpp_to_jpeg_para.resolution.width = width;
	dvpp_to_jpeg_para.is_align_image = true;
	ascend::utils::DvppProcess dvpp_to_jpeg(dvpp_to_jpeg_para);


	// call DVPP
	ascend::utils::DvppOutput dvpp_output;
	int32_t ret = dvpp_to_jpeg.DvppOperationProc(reinterpret_cast<char*>(image_handle->image_info.data.get()),
		img_size, &dvpp_output);

	// failed, no need to send to presenter
	if (ret != kDvppOperationOk) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
			"Failed to convert YUV420SP to JPEG, skip it.");
		return false;
	}

	// reset the data in img_vec
	jpeg_image.data.reset(dvpp_output.buffer, default_delete<uint8_t[]>());
	jpeg_image.size = dvpp_output.size;

	INFO_LOG("xxxxx convert image ok, size %d", jpeg_image.size);

	return true;
}

bool GeneralInference::PreProcess(const shared_ptr<EngineTrans> &image_handle,
                                  ImageData<u_int8_t> &resized_image) {
  // call ez_dvpp to resize image
  DvppBasicVpcPara resize_para;
  resize_para.input_image_type = INPUT_YUV420_SEMI_PLANNER_UV;
  resize_para.output_image_type = OUTPUT_YUV420SP_UV;

  // get original image size and set to resize parameter
  //int32_t width = ALIGN_UP(image_handle->image_info.width, DVPP_STRIDE_WIDTH);
  //int32_t height = ALIGN_UP(image_handle->image_info.height, DVPP_STRIDE_HEIGHT);

  int32_t width = image_handle->image_info.width;
  int32_t height = image_handle->image_info.height;

  // set source resolution ratio
  resize_para.src_resolution.width = width;
  resize_para.src_resolution.height = height;

  // crop parameters, only resize, no need crop, so set original image size
  // set crop left-top point (need even number)
  resize_para.crop_left = 0;
  resize_para.crop_up = 0;
  // set crop right-bottom point (need odd number)
  uint32_t crop_right = ((width >> 1) << 1) - 1;
  uint32_t crop_down = ((height >> 1) << 1) - 1;
  resize_para.crop_right = crop_right;
  resize_para.crop_down = crop_down;

  // set destination resolution ratio (need even number)
  uint32_t dst_width = (resize_width >> 1) << 1;
  uint32_t dst_height = (resize_height >> 1) << 1;
  resize_para.dest_resolution.width = dst_width;
  resize_para.dest_resolution.height = dst_height;

  // set input image align or not
  resize_para.is_input_align = true;
  resize_para.is_output_align = false;

  // call
  DvppProcess dvpp_resize_img(resize_para);
  // DVPP的输出结构体
  DvppVpcOutput dvpp_output;
  int ret = dvpp_resize_img.DvppBasicVpcProc(
      image_handle->image_info.data.get(), image_handle->image_info.size,
      &dvpp_output);
  
  if (ret != kDvppOperationOk) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call ez_dvpp failed, failed to resize image.");
    return false;
  }

  // call success, set data and size
  resized_image.data.reset(dvpp_output.buffer, default_delete<u_int8_t[]>());
  resized_image.size = dvpp_output.size;
  resized_image.width = dst_width;
  resized_image.height = dst_height;
  
  return true;
}

bool GeneralInference::Inference(
    const ImageData<u_int8_t> &resized_image,
    vector<shared_ptr<hiai::IAITensor>> &output_data_vec) {
  // neural buffer
  shared_ptr<hiai::AINeuralNetworkBuffer> neural_buf = nullptr;
  MAKE_SHARED_NO_THROW(neural_buf, hiai::AINeuralNetworkBuffer);
  if (neural_buf == nullptr) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "new AINeuralNetworkBuffer failed");
    return false;
  }
  neural_buf->SetBuffer((void*) resized_image.data.get(), resized_image.size);

  // input data
  shared_ptr<hiai::IAITensor> input_data = static_pointer_cast<hiai::IAITensor>(
      neural_buf);
  vector<shared_ptr<hiai::IAITensor>> input_data_vec;
  input_data_vec.push_back(input_data);

  // Call Process
  // 1. create output tensor
  hiai::AIContext ai_context;
  hiai::AIStatus ret = ai_model_manager_->CreateOutputTensor(input_data_vec,
                                                             output_data_vec);
  // create failed
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call CreateOutputTensor failed");
    return false;
  }

  // 2. process
  HIAI_ENGINE_LOG("aiModelManager->Process start!");
  ret = ai_model_manager_->Process(ai_context, input_data_vec, output_data_vec,
                                   kAiModelProcessTimeout);
  // process failed, also need to send data to post process
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "call Process failed");
    return false;
  }
  HIAI_ENGINE_LOG("xxxxx aiModelManager->Process end!");
  return true;
}

bool GeneralInference::SendToEngine(
    const shared_ptr<EngineTrans> &image_handle) {
  // can not discard when queue full
  HIAI_StatusT hiai_ret;
  do {
    hiai_ret = SendData(kSendDataPort, "EngineTrans",
                        static_pointer_cast<void>(image_handle));
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
  return true;
}

void GeneralInference::SendError(const std::string &err_msg,
                                 std::shared_ptr<EngineTrans> &image_handle) {
  image_handle->err_msg.error = true;
  image_handle->err_msg.err_msg = err_msg;
  if (!SendToEngine(image_handle)) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "SendData err_msg failed");
  }
}

bool GeneralInference::SendResult(
    shared_ptr<EngineTrans> &image_handle,
    vector<shared_ptr<hiai::IAITensor>> &output_data_vec) {

  for (uint32_t i = 0; i < output_data_vec.size(); i++) {
    shared_ptr<hiai::AISimpleTensor> result_tensor = static_pointer_cast<
        hiai::AISimpleTensor>(output_data_vec[i]);
    Output out;
    out.size = result_tensor->GetSize();
    out.data = std::shared_ptr<uint8_t>(new (nothrow) uint8_t[out.size],
                                        std::default_delete<uint8_t[]>());
    if (out.data == nullptr) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "dealing results: new array failed");
      continue;
    }
    errno_t mem_ret = memcpy_s(out.data.get(), out.size,
                               result_tensor->GetBuffer(),
                               result_tensor->GetSize());
    // memory copy failed, skip this result
    if (mem_ret != EOK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "dealing results: memcpy_s() error=%d", mem_ret);
      continue;
    }
    image_handle->inference_res.emplace_back(out);
  }

  ImageData<u_int8_t> jpeg_image;
  if (ConvertImage(image_handle, jpeg_image)) {
	  image_handle->image_info.data = jpeg_image.data;
	  image_handle->image_info.size = jpeg_image.size;
	  INFO_LOG("xxxxx add jpeg image to result data, image size %d", jpeg_image.size);
  }

  return SendToEngine(image_handle);
}



HIAI_IMPL_ENGINE_PROCESS("general_inference",
    GeneralInference, INPUT_SIZE) {
  HIAI_StatusT ret = HIAI_OK;

  // arg0 is empty
  if (arg0 == nullptr) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "arg0 is empty.");
    return HIAI_ERROR;
  }

  // just send data when finished
  shared_ptr<EngineTrans> image_handle = static_pointer_cast<EngineTrans>(arg0);
  if (image_handle->is_finished) {
    if (SendToEngine(image_handle)) {
      return HIAI_OK;
    }
    SendError("Failed to send finish data. Reason: Inference SendData failed.",
              image_handle);
    return HIAI_ERROR;
  }
    
  // resize image
  ImageData<u_int8_t> resized_image;
  if (!PreProcess(image_handle, resized_image)) {
    string err_msg = "Failed to deal file=" + image_handle->image_info.path
        + ". Reason: resize image failed.";
    SendError(err_msg, image_handle);
    return HIAI_ERROR;
  }
  image_handle->inference_width = resized_image.width;
  image_handle->inference_height = resized_image.height;
  // inference
  vector<shared_ptr<hiai::IAITensor>> output_data;
  if (!Inference(resized_image, output_data)) {
    string err_msg = "Failed to deal file=" + image_handle->image_info.path
        + ". Reason: inference failed.";
    SendError(err_msg, image_handle);
    return HIAI_ERROR;
  }

  // send result
  if (!SendResult(image_handle, output_data)) {
    string err_msg = "Failed to deal file=" + image_handle->image_info.path
        + ". Reason: Inference SendData failed.";
    SendError(err_msg, image_handle);
    return HIAI_ERROR;
  }
  return HIAI_OK;
}
