/**
* @file vdec_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef _VDEC_PROCESS_H_
#define _VDEC_PROCESS_H_

#include <cstdint>
#include <iostream>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

enum VDEC_STATUS {
    VDEC_UNINITED = 0,
    VDEC_INITED,
    VDEC_DECODE_FINISH,
    VDEC_REPORT_THREAD_EXIT,
    VDEC_EXIT_READY,
};
/**
 * VdecProcess
 */
class VdecProcess {
public:
    VdecProcess(int channel, uint32_t width, uint32_t height, 
                int type, aclvdecCallback callback);
    ~VdecProcess();

    static void* subscibe_report_thread_func(void *arg);

    Result init();
    Result init_resource();
    void destroy_resource();
    Result process(shared_ptr<FrameData> frameData, void* userData);

    void notify_finish() { 
        status_ = VDEC_DECODE_FINISH; 
    }
    bool is_decode_finish() { 
        return status_ == VDEC_DECODE_FINISH; 
    }
    bool is_exit_ready() { 
        return status_ == VDEC_EXIT_READY; 
    };
    bool is_init_ok() { 
        return status_ >= VDEC_INITED; 
    };
    void set_status(VDEC_STATUS status) { 
        status_ = status; 
    }
    Result set_width(uint32_t width);
    Result set_hight(uint32_t height);
    Result set_en_type(uint32_t enType);
    Result set_format(uint32_t format);
    Result video_param_check();

private:
    Result creat_vdec_channel_desc();
    Result create_input_stream_desc(shared_ptr<FrameData> frame);
    Result create_output_pic_desc(size_t size);

private:
    int status_;
    int channelId_;

    /* 1：YUV420 semi-planner（nv12）
       2：YVU420 semi-planner（nv21）
    */
    uint32_t format_; 

    /* 0：H265 main level
     * 1：H264 baseline level
     * 2：H264 main level
     * 3：H264 high level
     */
    uint32_t enType_;

    uint32_t frameWidth_;
    uint32_t frameHeight_;
    uint32_t alignWidth_;
    uint32_t alignHeight_;
    uint32_t outputPicSize_;
    uint32_t outputBufSize_;
    void *outputPicBuf_;
    aclvdecCallback callback_;
    
    aclrtStream stream_;
    aclrtRunMode runMode_; //运行模式,即当前应用运行在atlas200dk还是AI1

    aclvdecChannelDesc *vdecChannelDesc_;
    acldvppStreamDesc *inputStreamDesc_;
    acldvppPicDesc *outputPicDesc_;

    pthread_t subscribeThreadId_;

    bool isReleased_;
    
};

#endif
