/**
 * ============================================================================
 *
 * Copyright (C) 2018-2020, Hisilicon Technologies Co., Ltd. All Rights Reserved.
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

#ifndef ASCENDDK_ASCENDCAMERA_PARAMETER_UTILS_H_
#define ASCENDDK_ASCENDCAMERA_PARAMETER_UTILS_H_

#include <string>

namespace ascend {

namespace ascendcamera {

/*
 * ParameterUtils: is a util class used for handle the input parameters
 * */
class ParameterUtils {
 public:
  /**
   * @brief check parameter value is start with dash or not
   * @param [in] param_name: input parameter value
   * @return true: parameter value is start with dash;
   *         false: parameter value is not start with dash
   */
  static const bool IsStartWithDash(const std::string &param_name);

  /**
   * @brief check the file(this->outputFile) is occupied by another program
   * @param [in] output_file: output to file value
   * @return true: occupied by another program;
   *         false: not occupied by another program
   */
  static const bool CheckFileOccupied(const std::string &output_file);

  /**
   * @brief obtain file absolute directory
   * @param [out] file_path: file absolute directory
   * @return true: success to obtain file absolute directory;
   *         false: fail to obtain file absolute directory
   */
  static const bool ObtainFileAbsoluteDir(std::string &file_path);

  /**
   * @brief verify file name
   * @param [in] is_image: media type is image
   * @param [in] file_name: file name
   * @param [in] output_file: output to file value
   * @return true: verify pass; false: verify not pass
   */
  static const bool VerifyFileName(const bool is_image,
                                   const std::string &file_name,
                                   const std::string &output_file);

  /**
   * @brief overwrite the existing file
   * @param [in] output_file: output to file value
   * @return true: Overwrite the existing file success;
   *         false: Overwrite the existing file failed
   */
  static const bool OverwriteExistFile(const std::string &output_file);

  /**
   * @brief verify file directory
   * @param [in] file_path: file directory
   * @return true: verify pass; false: verify not pass
   */
  static const bool VerifyFileDir(const std::string &file_path);
};

}
}

#endif /* ASCENDDK_ASCENDCAMERA_PARAMETER_UTILS_H_ */
