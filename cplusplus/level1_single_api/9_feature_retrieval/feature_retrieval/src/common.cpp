/**
* @file common.cpp
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "common.h"
#include <string.h>

static bool ShortFeaDataRand(uint8_t* feaIn, size_t len, uint32_t* seqNum)
{
    // check data length
    if (len == 0) {
        ERROR_LOG("data length is 0!");
        return false;
    }

    if (len == BASESHORT_D_LEN) {
        // random base short data of range 0~255
        for (size_t i = SHORT_FEA_D_START; i < len; i++) {
            feaIn[i] = rand() % UINT8_DATA_LEN;
        }
    } else {
        // random base short data of range 0~255
        memcpy((feaIn + SHORT_FEA_D_SEQ_START_LONG), seqNum, sizeof(uint32_t));
        for (size_t i = SHORT_FEA_D_START_LONG; i < len; i++) {
            feaIn[i] = rand() % UINT8_DATA_LEN;
        }
    }
    return true;
}

static bool ShortFeaGen(uint8_t* feaIn, uint32_t baseNum, uint8_t feaType)
{
    size_t feaLen;
    if (feaType == 0) {
        feaLen = BASESHORT_D_LEN;
    } else {
        feaLen = BASESHORT_D_LEN_LONG;
    }
    INFO_LOG("feaLen is %zu.", feaLen);
    for (uint32_t i = 0; i < baseNum; i++) {
        auto ret = ShortFeaDataRand(feaIn + (i * feaLen), feaLen, &i);
        if (!ret) {
            ERROR_LOG("short feature data random failed!");
            return false;
        }
    }

    return true;
}


void *BaseShortFeaAlloc(uint32_t seed, uint32_t baseNum, uint8_t feaType)
{
    // set rand seed
    srand(seed);
    void* baseShortFea = nullptr;
    size_t mBaseDataLen = (feaType == 0) ? BASESHORT_D_LEN * baseNum :
                                BASESHORT_D_LEN_LONG * baseNum;
    // alloc M base memory
    auto ret = aclrtMallocHost(&baseShortFea, mBaseDataLen);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("alloc mem for baseShortFea failed!");
        return nullptr;
    }

    // init base short data with 0
    memset(baseShortFea, 0, mBaseDataLen);

    // generate short feature of M Base
    ret = ShortFeaGen(static_cast<uint8_t *>(baseShortFea), baseNum, feaType);
    if (!ret) {
        ERROR_LOG("copy short data to base failed!");
        free(baseShortFea);
        return nullptr;
    }

    return baseShortFea;
}

static bool AdcDataRand(float* adcDin, size_t len)
{
    // check data length
    if (len == 0) {
        ERROR_LOG("data length is 0!");
        return false;
    }

    // random adc data of range 1~1900
    for (size_t i = 0; i < len; i++) {
        adcDin[i] = ((float)((rand() % (ADC_RANDOM_MAX - ADC_RANDOM_MIN)) + ADC_RANDOM_MIN))  / ADC_RANDOM_REDUCTION;
    }

    return true;
}

void* AdcTabInit(uint32_t seed, size_t adcLen)
{
    // set rand seed
    srand(seed);

    // alloc table buffer
    void *adcTab = nullptr;
    auto ret = aclrtMallocHost(&adcTab, adcLen);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("alloc adcTab failed.");
        return nullptr;
    }

    // init adc table data with 0
    memset(adcTab, 0,  adcLen);

    // generate adc table data
    size_t adcRandomLen = adcLen / sizeof(float);
    auto randRet = AdcDataRand(static_cast<float *>(adcTab), adcRandomLen);
    if (!randRet) {
        ERROR_LOG("AdcDataRand failed!");
        free(adcTab);
        return nullptr;
    }

    return adcTab;
}
