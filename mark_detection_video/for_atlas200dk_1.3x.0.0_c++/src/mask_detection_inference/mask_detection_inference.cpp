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
#include "mask_detection_inference.h"
#include <vector>
#include "hiaiengine/log.h"
#include "hiaiengine/ai_types.h"
#include "hiaiengine/ai_model_parser.h"
#include "ascenddk/ascend_ezdvpp/dvpp_data_type.h"
#include "ascenddk/ascend_ezdvpp/dvpp_process.h"

using ascend::utils::DvppBasicVpcPara;
using ascend::utils::DvppProcess;
using ascend::utils::DvppVpcOutput;
using hiai::Engine;
using hiai::ImageData;
using hiai::IMAGEFORMAT;

namespace {
// output port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

// mask detection model input width
const uint32_t kModelWidth = 640;

// mask detection model input height
const uint32_t kModelHeight = 352;

// call dvpp success
const uint32_t kDvppProcSuccess = 0;
// level for call DVPP
const int32_t kDvppToJpegLevel = 100;

// vpc input image offset
const uint32_t kImagePixelOffsetEven = 1;
const uint32_t kImagePixelOffsetOdd = 2;
}

// register custom data type
HIAI_REGISTER_DATA_TYPE("EngineTransT", EngineTransT);
HIAI_REGISTER_DATA_TYPE("OutputT", OutputT);
HIAI_REGISTER_DATA_TYPE("ScaleInfoT", ScaleInfoT);
HIAI_REGISTER_DATA_TYPE("NewImageParaT", NewImageParaT);
HIAI_REGISTER_DATA_TYPE("BatchImageParaWithScaleT", BatchImageParaWithScaleT);

FaceDetectionInference::FaceDetectionInference() {
  ai_model_manager_ = nullptr;
}

HIAI_StatusT FaceDetectionInference::Init(
    const hiai::AIConfig& config,
    const std::vector<hiai::AIModelDescription>& model_desc) {
  HIAI_ENGINE_LOG("Start initialize!");

  // initialize aiModelManager
  if (ai_model_manager_ == nullptr) {
    ai_model_manager_ = std::make_shared<hiai::AIModelManager>();
  }

  // get parameters from graph.config
  // set model path and passcode to AI model description
  hiai::AIModelDescription fd_model_desc;
  for (int index = 0; index < config.items_size(); index++) {
    const ::hiai::AIConfigItem& item = config.items(index);
    // get model path
    if (item.name() == "model_path") {
      const char* model_path = item.value().data();
      fd_model_desc.set_path(model_path);
    }
  }

  // initialize model manager
  std::vector<hiai::AIModelDescription> model_desc_vec;
  model_desc_vec.push_back(fd_model_desc);
  hiai::AIStatus ret = ai_model_manager_->Init(config, model_desc_vec);
  // initialize AI model manager failed
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE, "initialize AI model failed");
    return HIAI_ERROR;
  }

  HIAI_ENGINE_LOG("End initialize!");
  return HIAI_OK;
}

HIAI_StatusT FaceDetectionInference::ImagePreProcess(
    const ImageData<u_int8_t>& src_img, ImageData<u_int8_t>& resized_img) {
  if (src_img.format != IMAGEFORMAT::YUV420SP) {
    // input image must be yuv420sp nv12.
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "[ODInferenceEngine] input image type does not match");
    return HIAI_ERROR;
  }

  // assemble resize param struct
  DvppBasicVpcPara dvpp_resize_param;
  dvpp_resize_param.input_image_type = INPUT_YUV420_SEMI_PLANNER_UV;
  dvpp_resize_param.src_resolution.height = src_img.height;
  dvpp_resize_param.src_resolution.width = src_img.width;

  // the value of crop_right and crop_left must be odd.
  dvpp_resize_param.crop_right =
      src_img.width % 2 == 0 ? src_img.width - kImagePixelOffsetEven :
      src_img.width-kImagePixelOffsetOdd;
  dvpp_resize_param.crop_down =
      src_img.height % 2 == 0 ? src_img.height - kImagePixelOffsetEven :
      src_img.height-kImagePixelOffsetOdd;

  dvpp_resize_param.dest_resolution.width = kModelWidth;
  dvpp_resize_param.dest_resolution.height = kModelHeight;

  // the input image is aligned in memory.
  dvpp_resize_param.is_input_align = true;

  DvppProcess dvpp_process(dvpp_resize_param);

  DvppVpcOutput dvpp_out;
  int ret = dvpp_process.DvppBasicVpcProc(src_img.data.get(),
      src_img.size, &dvpp_out);

  if (ret != kDvppProcSuccess) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call dvpp resize failed with code %d!",ret);
    return HIAI_ERROR;
  }

  // dvpp_out->pbuf
  resized_img.data.reset(dvpp_out.buffer, default_delete<uint8_t[]>());
  resized_img.size = dvpp_out.size;

  return HIAI_OK;
}

bool FaceDetectionInference::IsSupportFormat(hiai::IMAGEFORMAT format) {
  return format == hiai::YUV420SP;
}

HIAI_StatusT FaceDetectionInference::ConvertImage(NewImageParaT& img) {
  hiai::IMAGEFORMAT format = img.img.format;
  if (!IsSupportFormat(format)){
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Format %d is not supported!", format);
    return HIAI_ERROR;
  }

  uint32_t width = img.img.width;
  uint32_t height = img.img.height;
  uint32_t img_size = img.img.size;

  // parameter
  ascend::utils::DvppToJpgPara dvpp_to_jpeg_para;
  dvpp_to_jpeg_para.format = JPGENC_FORMAT_NV12;
  dvpp_to_jpeg_para.level = kDvppToJpegLevel;
  dvpp_to_jpeg_para.resolution.height = height;
  dvpp_to_jpeg_para.resolution.width = width;
  ascend::utils::DvppProcess dvpp_to_jpeg(dvpp_to_jpeg_para);
  

  // call DVPP
  ascend::utils::DvppOutput dvpp_output;
  int32_t ret = dvpp_to_jpeg.DvppOperationProc(reinterpret_cast<char*>(img.img.data.get()),
                                                img_size, &dvpp_output);

  // failed, no need to send to presenter
  if (ret != kDvppProcSuccess) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Failed to convert YUV420SP to JPEG, skip it.");
    return HIAI_ERROR;
  }

  // reset the data in img_vec
  img.img.data.reset(dvpp_output.buffer, default_delete<uint8_t[]>());
  img.img.size = dvpp_output.size;

  return HIAI_OK;
}


HIAI_IMPL_ENGINE_PROCESS("mask_detection_inference",
    FaceDetectionInference, INPUT_SIZE) {
  HIAI_ENGINE_LOG("Start process!");

  // check arg0 is null or not
  if (arg0 == nullptr) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Failed to process invalid message.");
    return HIAI_ERROR;
  }

  std::shared_ptr<BatchImageParaWithScaleT> image_handle =
      std::static_pointer_cast<BatchImageParaWithScaleT>(arg0);

  // initialize as zero
  uint32_t all_input_size = 0;
  vector<ImageData<u_int8_t>> processed_imgs;
  for (uint32_t i = 0; i < image_handle->b_info.batch_size; i++) {
    // image pre process
    ImageData<u_int8_t> resized_img;
    HIAI_StatusT vpc_ret =
        ImagePreProcess(image_handle->v_img[i].img, resized_img);
    if(vpc_ret != HIAI_OK){
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "image pre process error");
      continue;
    }
    all_input_size += resized_img.size * sizeof(uint8_t);
    processed_imgs.push_back(resized_img);
  }
  // input size is less than zero, do not need to inference
  if (all_input_size <= 0) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "all input image size=%u is less than zero",
                    all_input_size);
    return HIAI_ERROR;
  }

  // copy original data
  std::shared_ptr<EngineTransT> trans_data = std::make_shared<EngineTransT>();
  trans_data->b_info = image_handle->b_info;
  // convert the orginal image to JPEG
  for (uint32_t index = 0; index < image_handle->b_info.batch_size; index++){
    HIAI_StatusT convert_ret = ConvertImage(image_handle->v_img[index]);
    if (convert_ret != HIAI_OK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Convert YUV Image to Jpeg failed!");
      return HIAI_ERROR;
    }
  }
  trans_data->imgs = image_handle->v_img;


  // copy image data
  std::shared_ptr<uint8_t> temp = std::shared_ptr<uint8_t>(
      new uint8_t[all_input_size], std::default_delete<uint8_t[]>());
  uint32_t last_size = 0;
  for (uint32_t i = 0; i < processed_imgs.size(); i++) {

    // copy memory according to each size
    uint32_t each_size = processed_imgs[i].size * sizeof(uint8_t);
    HIAI_ENGINE_LOG("each input image size: %u", each_size);
    errno_t mem_ret = memcpy_s(temp.get() + last_size,
                               all_input_size - last_size,
                               processed_imgs[i].data.get(),
                               each_size);
    // memory copy failed, no need to inference, send original image
    if (mem_ret != EOK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "prepare image data: memcpy_s() error=%d", mem_ret);
      trans_data->status = false;
      trans_data->msg = "HiAIInference Engine memcpy_s image data failed";
      // send data to engine output port 0
      SendData(kSendDataPort, "EngineTransT",
               std::static_pointer_cast<void>(trans_data));
      return HIAI_ERROR;
    }
    last_size += each_size;
  }

  // neural buffer
  std::shared_ptr<hiai::AINeuralNetworkBuffer> neural_buf = std::shared_ptr<
      hiai::AINeuralNetworkBuffer>(
      new hiai::AINeuralNetworkBuffer(),
      std::default_delete<hiai::AINeuralNetworkBuffer>());
  neural_buf->SetBuffer((void*) temp.get(), all_input_size);

  // input data
  std::shared_ptr<hiai::IAITensor> input_data = std::static_pointer_cast<
      hiai::IAITensor>(neural_buf);
  std::vector<std::shared_ptr<hiai::IAITensor>> input_data_vec;
  input_data_vec.push_back(input_data);

  // Call Process
  // 1. create output tensor
  hiai::AIContext ai_context;
  std::vector<std::shared_ptr<hiai::IAITensor>> output_data_vector;
  hiai::AIStatus ret = ai_model_manager_->CreateOutputTensor(
      input_data_vec, output_data_vector);
  // create failed, also need to send data to post process
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "failed to create output tensor");
    trans_data->status = false;
    trans_data->msg = "HiAIInference Engine CreateOutputTensor failed";
    // send data to engine output port 0
    SendData(kSendDataPort, "EngineTransT",
             std::static_pointer_cast<void>(trans_data));
    return HIAI_ERROR;
  }

  // 2. process
  HIAI_ENGINE_LOG("aiModelManager->Process start!");
  ret = ai_model_manager_->Process(ai_context, input_data_vec,
                                   output_data_vector,
                                   AI_MODEL_PROCESS_TIMEOUT);
  // process failed, also need to send data to post process
  if (ret != hiai::SUCCESS) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "failed to process ai_model");
    trans_data->status = false;
    trans_data->msg = "HiAIInference Engine Process failed";
    // send data to engine output port 0
    SendData(kSendDataPort, "EngineTransT",
             std::static_pointer_cast<void>(trans_data));
    return HIAI_ERROR;
  }
  HIAI_ENGINE_LOG("aiModelManager->Process end!");

  // generate output data
  trans_data->status = true;
  for (uint32_t i = 0; i < output_data_vector.size(); i++) {
    std::shared_ptr<hiai::AISimpleTensor> result_tensor =
        std::static_pointer_cast<hiai::AISimpleTensor>(output_data_vector[i]);
    OutputT out;
    out.size = result_tensor->GetSize();
    out.data = std::shared_ptr<uint8_t>(new uint8_t[out.size],
                                        std::default_delete<uint8_t[]>());
    errno_t mem_ret = memcpy_s(out.data.get(), out.size,
                               result_tensor->GetBuffer(),
                               result_tensor->GetSize());
    // memory copy failed, skip this result
    if (mem_ret != EOK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "dealing results: memcpy_s() error=%d", mem_ret);
      continue;
    }
    trans_data->output_datas.push_back(out);
  }

  // send results and original image data to post process (port 0)
  HIAI_StatusT hiai_ret = SendData(kSendDataPort, "EngineTransT",
                                   std::static_pointer_cast<void>(trans_data));
  HIAI_ENGINE_LOG("End process!");
  return hiai_ret;
}
