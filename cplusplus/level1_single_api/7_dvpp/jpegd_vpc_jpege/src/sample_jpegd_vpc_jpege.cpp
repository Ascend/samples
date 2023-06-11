/**
 *  Copyright [2023] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

#include "hi_dvpp.h"
#include "acl.h"
#include "acl_rt.h"
#include "sample_comm.h"

using namespace std;

hi_s32 g_chn_id = 0;
hi_vpc_pic_info  jpegd_vpc_pic{};
hi_vpc_pic_info  vpc_jpege_pic{};
hi_pixel_format g_resize_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
hi_u32 g_resize_width = 964;
hi_u32 g_resize_height = 540;
string g_input_file_name = "dvpp_jpegd_decode_1920x1080.jpg";
string g_jpege_out_file_name = "jpege_out.jpg";
aclrtRunMode g_run_mode = ACL_DEVICE;
aclrtContext g_context = nullptr;

int32_t setup_acl_device()
{
    if (g_run_mode != ACL_HOST) {
        return HI_SUCCESS;
    }

    aclError aclRet = aclInit(nullptr);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclInit fail with %d.\n", aclRet);
        return aclRet;
    }
    SAMPLE_PRT("aclInit succ.\n");

    aclRet = aclrtSetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtSetDevice(0) fail with %d.\n", aclRet);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("aclrtSetDevice(0) succ.\n");

    aclRet = aclrtCreateContext(&g_context, 0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("acl create context failed with %d.\n", aclRet);
        aclrtResetDevice(0);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("create context success\n");

    aclRet = aclrtGetCurrentContext(&g_context);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("get current context failed\n");
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(0);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("get current context success\n");

    return HI_SUCCESS;
}

void destroy_acl_device()
{
    if (g_run_mode != ACL_HOST) {
        return;
    }

    if (g_context) {
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(0);
        aclFinalize();
    }
}
int32_t set_param()
{
    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet == ACL_SUCCESS) {
        if (g_run_mode == ACL_HOST) {
            SAMPLE_PRT(" Running in Host!\n");
        } else if (g_run_mode == ACL_DEVICE) {
            SAMPLE_PRT(" Running in Device!\n");
        } else {
            SAMPLE_PRT(" Running in Invalid platform! runMode:%u\n", g_run_mode);
            return HI_FAILURE;
        }
    } else {
        SAMPLE_PRT(" Get run mode fail! acl ret:%#x\n", aclRet);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

int32_t jpegd_process()
{
    hi_img_info stImgInfo{};
    hi_vdec_pic_info outPicInfo{};
    hi_vdec_stream stStream{};
    hi_video_frame_info stVFrame2{};
    hi_vdec_stream stStream2{};
    hi_vdec_supplement_info stSupplement{};
    int32_t s32ReadLen = 0;
    int32_t fileSize = 0;
    vector<uint8_t> fileData;
    FILE* fpStrm = nullptr;
    void *outBuffer = nullptr;
    void *inBuffer = nullptr;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return HI_FAILURE;
        }
    }
    // create vdec chn
    hi_vdec_chn_attr stChnAttr{};
    stChnAttr.type = HI_PT_JPEG;
    stChnAttr.mode = HI_VDEC_SEND_MODE_FRAME;
    stChnAttr.pic_width  = 8192; // max output 8192
    stChnAttr.pic_height = 8192; // max output 8192
    stChnAttr.stream_buf_size = 8192*8192; // max size 8192*8192
    stChnAttr.frame_buf_cnt  = 0;
    stChnAttr.frame_buf_size = 0;
    hi_s32 s32Ret = hi_mpi_vdec_create_chn(g_chn_id, &stChnAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("create vdec chn failed ret = %x\n", s32Ret);
        return HI_FAILURE;
    }

    // get and set chn param
    hi_vdec_chn_param stChnParam{};
    s32Ret = hi_mpi_vdec_get_chn_param(g_chn_id, &stChnParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vdec_get_chn_param failed ret = %x\n", s32Ret);
        goto DESTROY_CHN;
    }
    stChnParam.pic_param.pixel_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420; // it can be any jpegd output format
    stChnParam.pic_param.alpha = 255; // assign alpha value as 255
    stChnParam.display_frame_num = 0;
    s32Ret = hi_mpi_vdec_set_chn_param(g_chn_id, &stChnParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vdec_set_chn_param failed ret = %x\n", s32Ret);
        goto DESTROY_CHN;
    }

    // start recv stream
    s32Ret = hi_mpi_vdec_start_recv_stream(g_chn_id);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vdec_start_recv_stream failed ret = %x\n", s32Ret);
        goto DESTROY_CHN;
    }

    // open jpeg stream
    fpStrm = fopen(g_input_file_name.c_str(), "rb");
    if (fpStrm == nullptr) {
        SAMPLE_PRT(" can't open file %s !\n", g_input_file_name.c_str());
        goto DESTROY_CHN;
    }
    fseek(fpStrm, 0L, SEEK_END);
    fileSize = ftell(fpStrm);
    fflush(stdout);
    fseek(fpStrm, 0L, SEEK_SET);

    // malloc input stream on device
    s32Ret = hi_mpi_dvpp_malloc(0, (void**)&inBuffer, fileSize);
    if ((s32Ret != 0) || (inBuffer == nullptr)) {
        SAMPLE_PRT("[chn:%d] can't alloc %d in send stream thread!\n",
                   g_chn_id, fileSize);
        fclose(fpStrm);
        goto DESTROY_CHN;
    }

    if (g_run_mode == ACL_HOST) {
        fileData.resize(fileSize); // memory on host
        s32ReadLen = fread(&fileData[0], 1, fileSize, fpStrm);
        if (s32ReadLen == 0) {
            SAMPLE_PRT("cannot read stream");
            fclose(fpStrm);
            goto FAIL0;
        }
        auto aclRet = aclrtMemcpy(inBuffer, s32ReadLen, &fileData[0], s32ReadLen, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            fclose(fpStrm);
            SAMPLE_PRT("Copy host memcpy to device fail with %x.\n", aclRet);
            goto FAIL0;
        }
        stStream.addr = (uint8_t*)&fileData[0]; // use host addr
    } else {
        s32ReadLen = fread(inBuffer, 1, fileSize, fpStrm);
        if (s32ReadLen == 0) {
            SAMPLE_PRT("cannot read stream");
            fclose(fpStrm);
            goto FAIL0;
        }
        stStream.addr = (hi_u8*)inBuffer; // use device addr
    }
    fclose(fpStrm);

    // get the width, height and output image size of the stream
    stStream.pts = 0;
    stStream.len = s32ReadLen;
    stStream.end_of_frame  = HI_TRUE;
    stStream.end_of_stream = HI_FALSE;
    stStream.need_display  = HI_TRUE;
    // use HI_PIXEL_FORMAT_UNKNOWN to get the source format, or any output format jpegd supports
    s32Ret = hi_mpi_vdec_get_jpegd_output_info(&stStream, HI_PIXEL_FORMAT_UNKNOWN, &stImgInfo);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("get_image_info fail with %x.\n", s32Ret);
        goto FAIL0;
    }

    // malloc output image buffer on device
    s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, stImgInfo.img_buf_size);
    if (s32Ret != 0) {
        SAMPLE_PRT("hi_mpi_dvpp_malloc out buf Failed\n");
        goto FAIL0;
    }

    // start send stream
    stStream.addr = (uint8_t*)inBuffer;

    outPicInfo.width  = stImgInfo.width;
    outPicInfo.height = stImgInfo.height;
    outPicInfo.width_stride  = stImgInfo.width_stride;
    outPicInfo.height_stride = stImgInfo.height_stride;
    outPicInfo.buffer_size  = stImgInfo.img_buf_size;
    // use HI_PIXEL_FORMAT_UNKNOWN to decode as source format, or any output format jpegd supports
    outPicInfo.pixel_format = HI_PIXEL_FORMAT_UNKNOWN;
    outPicInfo.vir_addr = (hi_u64)outBuffer;
    s32Ret = hi_mpi_vdec_send_stream(g_chn_id, &stStream, &outPicInfo, -1);
    if (s32Ret != 0) {
        SAMPLE_PRT("send stream failed ret = 0x%x", s32Ret);
        goto FAIL1;
    }

    // get frame
    s32Ret = hi_mpi_vdec_get_frame(g_chn_id, &stVFrame2, &stSupplement, &stStream2, -1);
    if (s32Ret == HI_SUCCESS) {
        // check if success
        if (stVFrame2.v_frame.frame_flag == 0) {
            SAMPLE_PRT("[chn:%d] Yes, GetFrame Success.\n", g_chn_id);
        } else {
            SAMPLE_PRT("frame flag is not zero");
            goto FAIL1;
        }
    } else {
        SAMPLE_PRT("get frame failed ret = %x", s32Ret);
        goto FAIL1;
    }

    SAMPLE_PRT("jpegd out format %u, width %u, height %u \n",
        stVFrame2.v_frame.pixel_format, stVFrame2.v_frame.width, stVFrame2.v_frame.height);
    // free input memory on device
    if (inBuffer != nullptr) {
        hi_mpi_dvpp_free(inBuffer);
        inBuffer = nullptr;
    }

    s32Ret = hi_mpi_vdec_release_frame(g_chn_id, &stVFrame2);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("release frame failed ret = 0x%x", s32Ret);
        goto FAIL1;
    }
    s32Ret = hi_mpi_vdec_stop_recv_stream(g_chn_id);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("stop recv stream failed ret = 0x%x", s32Ret);
        goto FAIL1;
    }
    s32Ret = hi_mpi_vdec_destroy_chn(g_chn_id);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("destroy chn failed ret = 0x%x", s32Ret);
        goto FAIL1;
    }
    // copy jpegd output info to vpc input picture
    jpegd_vpc_pic.picture_width = stVFrame2.v_frame.width;
    jpegd_vpc_pic.picture_height = stVFrame2.v_frame.height;
    jpegd_vpc_pic.picture_width_stride = stVFrame2.v_frame.width_stride[0];
    jpegd_vpc_pic.picture_height_stride = stVFrame2.v_frame.height_stride[0];
    jpegd_vpc_pic.picture_format = stVFrame2.v_frame.pixel_format;
    jpegd_vpc_pic.picture_address = stVFrame2.v_frame.virt_addr[0];
    jpegd_vpc_pic.picture_buffer_size = outPicInfo.buffer_size;
    
    return HI_SUCCESS;
FAIL1:
    if (outBuffer != nullptr) {
        hi_mpi_dvpp_free((void*)outBuffer);
        outBuffer = nullptr;
    }
FAIL0:
    if (inBuffer != nullptr) {
        hi_mpi_dvpp_free((void*)inBuffer);
        inBuffer = nullptr;
    }
DESTROY_CHN:
    hi_mpi_vdec_destroy_chn(g_chn_id);
    return HI_FAILURE;
}

int32_t save_jpege_pic(hi_venc_stream &stream)
{
    FILE* fd = nullptr;
    fd = fopen(g_jpege_out_file_name.c_str(), "wb");
    if (fd == nullptr) {
        SAMPLE_PRT("open output file err\n");
        return HI_FAILURE;
    }
    if (g_run_mode == ACL_HOST) {
        uint32_t dataLen = stream.pack[0].len - stream.pack[0].offset;
        char* pcData = new char[dataLen];
        auto aclRet = aclrtMemcpy(pcData, dataLen, stream.pack[0].addr + stream.pack[0].offset,
            dataLen, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("aclrtMemcpy fail %d pcData:%lx,dataLen:%u,addr:%lx,offset:%d ",
                aclRet, (uint64_t)pcData, dataLen, (uint64_t)stream.pack[0].addr, stream.pack[0].offset);
            delete[] pcData;
            pcData = nullptr;
            return HI_FAILURE;
        }
        fwrite(pcData, dataLen, 1, fd);
        fflush(fd);
        delete[] pcData;
        pcData = nullptr;
    } else {
        fwrite(stream.pack[0].addr + stream.pack[0].offset,
            stream.pack[0].len - stream.pack[0].offset, 1, fd);
        fflush(fd);
    }
    fclose(fd);
    return HI_SUCCESS;
}

int32_t jpege_process()
{
    hi_venc_jpeg_param stParamJpeg{};
    void* outputBuf;
    hi_img_stream out_stream{};
    hi_venc_stream stream{};
    hi_video_frame_info inputFrame{};
    hi_u32 outputBufSize = 0;

    hi_venc_chn_attr vencChnAttr;
    vencChnAttr.venc_attr.type = HI_PT_JPEG;
    vencChnAttr.venc_attr.profile = 0;
    vencChnAttr.venc_attr.max_pic_width = vpc_jpege_pic.picture_width;
    vencChnAttr.venc_attr.max_pic_height = vpc_jpege_pic.picture_height;
    vencChnAttr.venc_attr.pic_width = vpc_jpege_pic.picture_width;
    vencChnAttr.venc_attr.pic_height = vpc_jpege_pic.picture_height;
    vencChnAttr.venc_attr.buf_size = 
        ALIGN_UP(vpc_jpege_pic.picture_width * vpc_jpege_pic.picture_width, 64); // aligned to 64 bytes
    vencChnAttr.venc_attr.is_by_frame = HI_TRUE; // get stream mode is field mode or frame mode
    vencChnAttr.venc_attr.jpeg_attr.dcf_en = HI_FALSE;
    vencChnAttr.venc_attr.jpeg_attr.recv_mode = HI_VENC_PIC_RECV_SINGLE;
    vencChnAttr.venc_attr.jpeg_attr.mpf_cfg.large_thumbnail_num = 0;
    // create venc chn
    int32_t s32Ret = hi_mpi_venc_create_chn(g_chn_id, &vencChnAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_create_chn [%d] faild with %#x!\n", g_chn_id, s32Ret);
        return s32Ret;
    }
    // start venc chn
    hi_venc_start_param recvParam;
    recvParam.recv_pic_num = -1; // unspecified frame count
    s32Ret = hi_mpi_venc_start_chn(g_chn_id, &recvParam);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_start_chn faild with%#x in chnl %d!\n", s32Ret, g_chn_id);
        goto DESTROY_CHN;
    }

    // set encode parameter
    s32Ret = hi_mpi_venc_get_jpeg_param(g_chn_id, &stParamJpeg);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_get_jpeg_param err 0x%x\n", s32Ret);
        goto DESTROY_CHN;
    }
    stParamJpeg.qfactor = 100; // assign qfactor as 100
    for (hi_u32 i = 0; i < HI_VENC_JPEG_QT_COEF_NUM; i++) {
        stParamJpeg.y_qt[i] = 0xFF;
        stParamJpeg.cb_qt[i] = 0xFF;
        stParamJpeg.cr_qt[i] = 0xFF;
    }
    s32Ret = hi_mpi_venc_set_jpeg_param(g_chn_id, &stParamJpeg);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_set_jpeg_param err 0x%x\n", s32Ret);
        goto DESTROY_CHN;
    }

    // get predicted stream size
    inputFrame.pool_id = 0;
    inputFrame.v_frame.width = vpc_jpege_pic.picture_width;
    inputFrame.v_frame.height = vpc_jpege_pic.picture_height;
    inputFrame.v_frame.dynamic_range = HI_DYNAMIC_RANGE_SDR8; // Dynamic Range
    inputFrame.v_frame.compress_mode = HI_COMPRESS_MODE_NONE; // Compression Mode
    inputFrame.v_frame.pixel_format = vpc_jpege_pic.picture_format;
    inputFrame.v_frame.video_format = HI_VIDEO_FORMAT_LINEAR; // Video format
    inputFrame.v_frame.field = HI_VIDEO_FIELD_FRAME; // Frame Or Field Mode
    inputFrame.v_frame.color_gamut = HI_COLOR_GAMUT_BT709; // Gamut range
    inputFrame.v_frame.header_stride[0] = 0; // Image compression head span
    inputFrame.v_frame.header_stride[1] = 0;
    inputFrame.v_frame.width_stride[0] = vpc_jpege_pic.picture_width_stride;
    inputFrame.v_frame.width_stride[1] = vpc_jpege_pic.picture_width_stride;
    inputFrame.v_frame.header_virt_addr[0] = vpc_jpege_pic.picture_address; // Compression header virtual address
    inputFrame.v_frame.header_virt_addr[1] = inputFrame.v_frame.header_virt_addr[0];
    // virtual address
    inputFrame.v_frame.virt_addr[0] = inputFrame.v_frame.header_virt_addr[0];
    inputFrame.v_frame.virt_addr[1] =
        (hi_void*)((uintptr_t)inputFrame.v_frame.virt_addr[0] +
        vpc_jpege_pic.picture_width_stride * vpc_jpege_pic.picture_height_stride);
    s32Ret = hi_mpi_venc_get_jpege_predicted_size(&inputFrame, &stParamJpeg, &outputBufSize);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_get_jpege_predicted_size err 0x%x\n", s32Ret);
        goto FAIL0;
    }

    // malloc output buffer
    s32Ret = hi_mpi_dvpp_malloc(0, &outputBuf, outputBufSize);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("malloc venc out buffer err 0x%x\n", s32Ret);
        goto FAIL0;
    }

    // send frame
    s32Ret = hi_mpi_venc_send_frame(g_chn_id, &inputFrame, 10000); // time out 10000us
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_send_jpege_frame err 0x%x\n", s32Ret);
        goto FAIL1;
    }

    stream.pack_cnt = 1;
    stream.pack = (hi_venc_pack*)malloc(sizeof(hi_venc_pack));
    if (stream.pack == nullptr) {
        SAMPLE_PRT("malloc failed!\n");
        goto FAIL1;
    }

    // get stream
    s32Ret = hi_mpi_venc_get_stream(g_chn_id, &stream, -1);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_get_stream failed with %#x!\n", s32Ret);
        goto FAIL2;
    }
    // save file
    s32Ret = save_jpege_pic(stream);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("save jpege pic failed with %#x!\n", s32Ret);
        goto FAIL2;
    }

    // release stream
    s32Ret = hi_mpi_venc_release_stream(g_chn_id, &stream);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_release_stream failed with %#x!\n", s32Ret);
        goto FAIL2;
    }

    s32Ret = hi_mpi_venc_stop_chn(g_chn_id);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_stop_chn vencChn[%d] failed with %#x!\n", g_chn_id, s32Ret);
        goto FAIL2;
    }
    s32Ret = hi_mpi_venc_destroy_chn(g_chn_id);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_destroy_chn [%d] failed with %#x!\n", g_chn_id, s32Ret);
        goto FAIL2;
    }

    // free memory
    hi_mpi_dvpp_free(outputBuf);
    outputBuf = nullptr;

    hi_mpi_dvpp_free(vpc_jpege_pic.picture_address);
    vpc_jpege_pic.picture_address = nullptr;

    free(stream.pack);
    stream.pack = nullptr;

    SAMPLE_PRT("jpegd_vpc_jpege sample finished\n");
    return HI_SUCCESS;
FAIL2:
    free(stream.pack);
    stream.pack = nullptr;
FAIL1:
    hi_mpi_dvpp_free(outputBuf);
    outputBuf = nullptr;
FAIL0:
    hi_mpi_dvpp_free(vpc_jpege_pic.picture_address);
    vpc_jpege_pic.picture_address = nullptr;
DESTROY_CHN:
    hi_mpi_venc_destroy_chn(g_chn_id);
    return HI_FAILURE;
}

hi_s32 vpc_process()
{
    hi_u32 taskID = 0;
    hi_vpc_chn_attr stChnAttr{};
    hi_img_base_info base_info{};
    hi_img_align_info align_info{};

    // get vpc output picture info
    base_info.width = g_resize_width;
    base_info.height = g_resize_height;
    base_info.pixel_format = g_resize_format;
    hi_module_type mod_type[2];
    mod_type[0] = HI_MOD_VPC;
    mod_type[1] = HI_MOD_VENC;
    hi_mpi_sys_get_image_align_info(&mod_type[0], 2, &base_info, &align_info); // array length is 2 

    // configure vpc_jpege pic structure
    vpc_jpege_pic.picture_width = g_resize_width;
    vpc_jpege_pic.picture_height = g_resize_height;
    vpc_jpege_pic.picture_width_stride = align_info.width_stride;
    vpc_jpege_pic.picture_height_stride = align_info.height_stride;
    vpc_jpege_pic.picture_buffer_size = align_info.img_buf_size;
    vpc_jpege_pic.picture_format = g_resize_format;

    // malloc vpc output buffer
    hi_s32 s32Ret = hi_mpi_dvpp_malloc(0, &vpc_jpege_pic.picture_address, vpc_jpege_pic.picture_buffer_size);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("buf alloc failed!\n");
        goto FAIL0;
    }
    // create chn
    s32Ret = hi_mpi_vpc_create_chn(g_chn_id, &stChnAttr);
    if (s32Ret != HI_SUCCESS) {
      SAMPLE_PRT("create vpc chn failed failed! ret %x\n", s32Ret);
      goto FAIL1;
    }

    // fx = 0, fy = 0, inerpolation = 0
    s32Ret = hi_mpi_vpc_resize(g_chn_id, &jpegd_vpc_pic, &vpc_jpege_pic, 0, 0, 0, &taskID, -1); // time out -1
    if(s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("vpc resize failed failed! ret %x\n", s32Ret);
        goto FAIL1;
    }
    // get result
    s32Ret = hi_mpi_vpc_get_process_result(g_chn_id, taskID, -1); // time out -1
    if(s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("vpc get result failed! ret %x\n", s32Ret);
        goto FAIL1;
    }

    // destroy chn
    s32Ret = hi_mpi_vpc_destroy_chn(g_chn_id);
    if(s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("vpc destroy failed! ret %x\n", s32Ret);
        goto FAIL1;
    }
    SAMPLE_PRT("vpc out format %u, width %u, height %u\n",
        vpc_jpege_pic.picture_format, vpc_jpege_pic.picture_width, vpc_jpege_pic.picture_height);
    
    // free jpegd_vpc memory
    hi_mpi_dvpp_free(jpegd_vpc_pic.picture_address);
    jpegd_vpc_pic.picture_address = nullptr;
    return HI_SUCCESS;

FAIL1:
    hi_mpi_dvpp_free(vpc_jpege_pic.picture_address);
    vpc_jpege_pic.picture_address = nullptr;
FAIL0:
    hi_mpi_dvpp_free(jpegd_vpc_pic.picture_address);
    jpegd_vpc_pic.picture_address = nullptr;

    hi_mpi_vpc_destroy_chn(g_chn_id);
    return HI_FAILURE;
}

int32_t main(int32_t argc, char *argv[])
{
    int32_t s32Ret = HI_SUCCESS;

    s32Ret = set_param();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("set_param failed!\n");
        return HI_FAILURE;
    }

    s32Ret = setup_acl_device();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("Setup Device failed! ret code:%#x\n", s32Ret);
        return HI_FAILURE;
    }

    s32Ret = hi_mpi_sys_init();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_init failed!\n");
        return HI_FAILURE;
    }

    s32Ret = jpegd_process();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("jpegd_send_stream failed!\n");
        return HI_FAILURE;
    }

    s32Ret = vpc_process();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("vpc_process failed!\n");
        return HI_FAILURE;
    }
    
    s32Ret = jpege_process();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("jpege_start failed!\n");
        return HI_FAILURE;
    }

    hi_mpi_sys_exit();

    destroy_acl_device();

    return 0;
}
