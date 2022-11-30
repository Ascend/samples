/**
* @file common.h
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef COMMON_H
#define COMMON_H

#include <iostream>

#include "acl/acl.h"

#define SUCCESS 0
#define FAILED 1

#define ACL_REQUIRES_SUCCESS(val, expr0, expr1) \
    do {                                        \
        if ((val) != ACL_SUCCESS) {             \
            expr0;                              \
            expr1;                              \
        }\
    } while (false)

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stderr, "[ERROR]  " fmt "\n", ##args)


#define UINT8_DATA_LEN 256
#define BASESHORT_D_LEN 36
#define BASESHORT_D_LEN_LONG 44
#define SHORT_FEA_D_START 4 // valid data position in 36B length data
#define SHORT_FEA_D_START_LONG 12
#define SHORT_FEA_D_SEQ_START_LONG SHORT_FEA_D_START

#define ADC_RANDOM_MAX 1900
#define ADC_RANDOM_MIN 1
#define ADC_RANDOM_REDUCTION 10000

/**
 * @brief alloc and set rand value for feature
 * @param [in] seed: rand seed
 * @param [in] baseNum: feature number
 * @param [in] feaType: feature typer
 * @return buffer
 */
void *BaseShortFeaAlloc(uint32_t seed, uint32_t baseNum, uint8_t feaType);

/**
 * @brief Init feature search input table
 * @param [in] seed: rand seed
 * @param [in] adcLen: table length
 * @return table buffer
 */
void* AdcTabInit(uint32_t seed, size_t adcLen);

#endif // COMMON_H
