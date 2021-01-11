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
#ifndef ENGINE_POST_DANGE_H_
#define ENGINE_POST_DANGE_H_
#include "common.h"
#include "engine_handle.h"

/**
 * @brief: common post_process
 */
class EnginePostDange{
public:
    static int _s_flag ;
  /**
   * @brief: construction function
   */
  EnginePostDange();

  /**
   * @brief: the destruction function
   */
  ~EnginePostDange() = default;

  /**
   * @brief: common post_process engine initialize
   * @param [in]: engine's parameters which configured in graph.config
   * @param [in]: model description
   * @return: HIAI_StatusT
   */

    int Init();

  /**
   * @brief: engine processor
   *         1. dealing results
   *         2. call OSD to draw box and label if needed
   *         3. call DVPP to change YUV420SP to JPEG
   *         4. call presenter agent to send JPEG to server
   * @param [in]: input size
   * @param [in]: output size
   */
    int HandleResults(float* Pdata,int size, int mode);
    int handle_preview(ImageData& result, int mode);

private:
    int common_shm_init(void);
    int hand_wheel(void);
    int Handle_dange_mode(float *result,int size);
    int Handle_off_mode(float *result,int size);
    int Handle_remote_mode(float *result,int size);
    EngineHandle EngineHan;
    //    i2c i2c_ctrl;
    //    wheel wheel_ctrl;


};

#endif /* COMMON_POST_DANGE_H_ */
