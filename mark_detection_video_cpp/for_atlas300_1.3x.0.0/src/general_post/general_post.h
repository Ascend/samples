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

#ifndef GENERAL_POST_GENERAL_POST_H_
#define GENERAL_POST_GENERAL_POST_H_

#include<vector>
#include "hiaiengine/engine.h"
#include "hiaiengine/data_type.h"
#include "data_type.h"
#include "ascenddk/presenter/agent/presenter_channel.h"

#define INPUT_SIZE 1
#define OUTPUT_SIZE 1

const static std::vector<string> label = {"person", "face", "mask"};

/**
 * @brief: inference engine class
 */
class GeneralPost : public hiai::Engine {
public:
  /**
   * @brief: engine initialize
   * @param [in]: engine's parameters which configured in graph.config
   * @param [in]: model description
   * @return: HIAI_StatusT
   */
  HIAI_StatusT Init(const hiai::AIConfig& config,
                    const std::vector<hiai::AIModelDescription>& model_desc);

  #define  sigmoid(x)  (1 / (1 + exp(-1*x)))

  /**
   * @brief: engine processor which override HIAI engine
   *         get every image, and then send data to inference engine
   * @param [in]: input size
   * @param [in]: output size
   */
  HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

private:
   bool IsInValidIp(const std::string& ip);
   bool IsInValidPort(int32_t port);
   bool IsInValidChannelName(const std::string& channel_name);


  std::vector<BoundingBox>  decodeTensor(const std::shared_ptr<EngineTrans> &result,uint ImgW, uint Imgh);

  std::vector<BoundingBox> nonMaximumSuppression(const float nmsThresh, std::vector<BoundingBox> binfo);

  std::vector<BoundingBox> nmsAllClasses(const float nmsThresh, std::vector<BoundingBox>& binfo, const uint numClasses);
  /**
   * @brief: send result
   * @return: true: success; false: failed
   */
  bool SendSentinel();

  /**
   * @brief: mark the oject based on detection result
   * @param [in]: result: engine transform image
   * @return: HIAI_StatusT
   */
  HIAI_StatusT SendImage(const ascend::presenter::ImageFrame& inference_result);

  HIAI_StatusT PostProcess(
      const std::shared_ptr<EngineTrans> &result);

  float confidence;
  std::string kOutputFilePrefix;
  std::string output_path;

  std::string presenter_ip;
  // presenter server port for agent
  int32_t presenter_port;
  // channel name
  std::string channel_name;
  // presenter channel
  std::shared_ptr<ascend::presenter::Channel> presenter_channel_;
};

#endif /* GENERAL_POST_GENERAL_POST_H_ */
