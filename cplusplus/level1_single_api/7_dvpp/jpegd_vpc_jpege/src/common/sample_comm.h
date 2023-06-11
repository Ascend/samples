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

#ifndef __SAMPLE_COMM_H__
#define __SAMPLE_COMM_H__

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

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif /* End of #ifndef __SAMPLE_COMMON_H__ */
