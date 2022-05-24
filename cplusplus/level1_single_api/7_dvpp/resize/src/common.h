/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
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

#ifndef __COMMON_H__
#define __COMMON_H__

#include "hi_dvpp.h"
#include "acl.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

/*******************************************************
    macro define
*******************************************************/
#define CHECK_CHN_RET(express, Chn, name) \
    do {  \
        int32_t Ret;  \
        Ret = express; \
        if (HI_SUCCESS != Ret) { \
            printf(" \033[0;31m%s chn %d failed at %s: LINE: %d with %#x! \033[0;39m \n", \
                name, Chn, __FUNCTION__, __LINE__, Ret); \
            fflush(stdout); \
            return Ret; \
        } \
    } while (0)

#define CHECK_RET(express, name) \
    do { \
        int32_t Ret; \
        Ret = express; \
        if (HI_SUCCESS != Ret) { \
            printf(" \033[0;31m%s failed at %s: LINE: %d with %#x! \033[0;39m \n", \
                name, __FUNCTION__, __LINE__, Ret); \
            return Ret; \
        } \
    } while (0)

#define PAUSE()  do { \
        printf("---------------press Enter key to exit!--------------- \n"); \
        getchar(); \
    } while (0)

#define SAMPLE_PRT(fmt...)    \
    do { \
        printf("[%s]-%d: ", __FUNCTION__, __LINE__); \
        printf(fmt); \
    } while (0)

#define CHECK_NULL_PTR(ptr) \
    do { \
        if (NULL == ptr) { \
            printf("func:%s, line:%d, NULL pointer \n", __FUNCTION__, __LINE__); \
            return HI_FAILURE; \
        } \
    } while (0)

constexpr uint32_t FILE_NAME_LEN = 500;
constexpr uint32_t MAX_MULTI_COUNT = 256;

struct VpcAttr {
    char inputFileName[FILE_NAME_LEN]{};
    char outputFileName[FILE_NAME_LEN]{};
    uint32_t width{3840}; // initialize the input width to 3840
    uint32_t height{2160}; // initialize the input width to 2160
    uint32_t format{0};
    uint32_t outWidth{1920}; // initialize the output width to 1920
    uint32_t outHeight{1080}; // initialize the output width to 1080
    uint32_t outFormat{0};
    double fx{0};
    double fy{0};
    int32_t interpolation{0};
};

struct FuncInput {
    hi_vpc_chn chnId{0};
    VpcAttr g_vpc_attribute{};
};

extern aclrtContext g_context;
extern aclrtRunMode g_run_mode;

/*******************************************************
    function announce
*******************************************************/
/*
* @brief : Get current run mode, acl host or acl device
* @return : 0: get success; -1: get failed
*/
int32_t get_run_mode();

/*
* @brief : initialize acl environment
* @return : 0: success; -1: failed
*/
int32_t acl_init();

/*
* @brief : deinitialization acl environment
* @return : 0: success; -1: fail
*/
int32_t acl_destory_resource();

/*
* @brief : malloc buffer in device
* @param [in] bufSize: applied buffer size
* @param [out] addrPtr: buffer address
* @return : 0: success; others: error code
*/
int32_t dvpp_mem_malloc(void** addrPtr, unsigned int bufSize);

/*
* @brief : free buffer in device
* @param [in] addrPtr: buffer address
* @return : 0: success; others: error code
*/
int32_t dvpp_mem_free(void* addr);

/*
* @brief : calculate picture stride and buffer size according to the picture format and vpc alignment requirements
* @param [in] pic: vpc input parameter struct：pictureWidth, picture_height, pictrueFormat, AlignNum
* @param [out] pic: vpc input parameter struct: picture_width_stride, picture_height_stride, picture_buffer_size
* @return : actual buffer size
*/
uint32_t configure_stride_and_buffer_size(hi_vpc_pic_info& pic, uint32_t widthAlign = 16,
    uint32_t heightAlign = 2, bool widthStride32Align = true);

/*
* @brief : get dest picture with target stride
* @param [in] srcPic: src stride pic
* @param [in] dstPic: dst stride pic
* @param [out] dstPic: dst stride pic
*/
int32_t get_dst_stride_picture(const hi_vpc_pic_info& srcPic, const hi_vpc_pic_info& dstPic);

/*
* @brief : malloc input buffer and read input data for both host and device scenario.
*          the input memory is released externally.
* @param [in] inputPic: vpc input parameter struct
* @param [in] inputFileName: input data file name
* @return : 0: success; others: error code
*/
int32_t prepare_input_data(hi_vpc_pic_info& inputPic, const char inputFileName[]);

/*
* @brief : write output data for both host and device scenario
* @param [in] outputPic: vpc output parameter struct
* @param [in] dstBufferSize: output data size(not aligned)
* @param [in] outputFileName: output data file name
* @return : 0: success; others: error code
*/
int32_t handle_output_data(hi_vpc_pic_info& outputPic, uint32_t dstBufferSize, const char outputFileName[]);

/*
* @brief : memset buffer for both host and device scenario
* @param [in] picInfo: vpc input/output parameter struct
*/
void memset_buffer(hi_vpc_pic_info& picInfo);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif /* End of #ifndef __COMMON_H__ */
