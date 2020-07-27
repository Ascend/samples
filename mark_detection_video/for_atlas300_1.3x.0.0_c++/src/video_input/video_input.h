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

#ifndef GENERAL_IMAGE_GENERAL_IMAGE_H_
#define GENERAL_IMAGE_GENERAL_IMAGE_H_

#include "hiaiengine/engine.h"
#include "hiaiengine/data_type.h"
#include "data_type.h"


#define INPUT_SIZE 1
#define OUTPUT_SIZE 1

struct VideoInfo {
	int format = 0;
	int channel_id = -1;
	void* context = NULL;
};

/**
 * @brief: inference engine class
 */
class VideoInput : public hiai::Engine {
public:
  /**
   * @brief: engine initialize
   * @param [in]: engine's parameters which configured in graph.config
   * @param [in]: model description
   * @return: HIAI_StatusT
   */
  HIAI_StatusT Init(const hiai::AIConfig& config,
                    const std::vector<hiai::AIModelDescription>& model_desc);

  /**
   * @brief: engine processor which override HIAI engine
   *         get every image, and then send data to inference engine
   * @param [in]: input size
   * @param [in]: output size
   */
  HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

  static void FrameDecodeCallback(void* context, void* frame_data, int frame_size);
private:

  /**
   * @brief: send result
   * @return: true: success; false: failed
   */
  bool SendSentinel();

  /**
   * @brief: arrange image information
   * @param [out]: image_handle: image handler
   * @param [in]: image file path
   * @return: true: success; false: failed
   */
  bool ArrangeImageInfo(std::shared_ptr<EvbImageInfo> &image_handle,
                        const std::string &image_path);

  /**
   * @brief: send result
   * @param [in]: image_handle: engine transform image
   * @return: true: success; false: failed
   */
  bool SendToEngine(const std::shared_ptr<EvbImageInfo> &image_handle);

  bool IsValidMp4File(const string &input_str);

  /**
   * @brief: get files from path and it's subpath
   * @param [in]: path can be liked as "path1,file,path2"
   * @param [out]: all existed files
   * @return: true: success; false: failed
   */
  void GetAllFiles(const std::string &path, std::vector<std::string> &file_vec);

  /**
   * @brief: path is directory or not
   * @param [in]: path
   * @return: true: directory; false: not directory
   */
  bool IsDirectory(const std::string &path);

  /**
   * @brief: path is exist or not
   * @param [in]: path
   * @return: true: exist; false: not exist
   */
  bool IsPathExist(const std::string &path);

  /**
   * @brief: split file path
   * @param [in]: path can be liked as "path1,file,path2"
   * @param [out]: split file paths
   */
  void SplitPath(const std::string &path, std::vector<std::string> &path_vec);

  /**
   * @brief: get files from one file path
   * @param [in]: path can be liked as "path1" or "path2"
   * @param [out]: files in this path
   */
  void GetPathFiles(const std::string &path,
                    std::vector<std::string> &file_vec);

  int DecodeVideo(const string& stream_name,
   	           int channel_id, const string& transport);

  std::string src_path_;
};

#endif /* GENERAL_IMAGE_GENERAL_IMAGE_H_ */
