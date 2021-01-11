/**
* @file gemm_runner.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef GEMM_RUNNER_H
#define GEMM_RUNNER_H

#include "acl/acl.h"

class GemmRunner {
public:
    /**
     * @brief Constructor
     * @param [in] m: number of rows of matrix A and C
     * @param [in] n: number of columns of matrix B and C
     * @param [in] k: number of columns of A and rows of B
     */
    GemmRunner(uint32_t m, uint32_t n, uint32_t k, aclDataType inputType, aclDataType outputType);

    /**
     * @brief Destructor
     */
    virtual ~GemmRunner();

    /**
     * @brief Malloc device memories
     * @return true: success; false: failure
     */
    bool Init();

    template<typename T>
    void SetAlpha(T alpha)
    {
        *reinterpret_cast<T *>(hostAlpha_) = alpha;
    }

    template<typename T>
    void SetBeta(T beta)
    {
        *reinterpret_cast<T *>(hostBeta_) = beta;
    }

    /**
     * @brief Read inputs from data file
     * @return true: success; false: failure
     */
    bool PrepareInputs();

    /**
     * @brief Launch gemm op
     * @return true: success; false: failure
     */
    bool RunOp();

    /**
     * @brief Write output to result file
     * @return true: success; false: failure
     */
    bool WriteOutput();

    /**
     * @brief Print readable data of matrix A
     */
    void PrintMatrixA();

    /**
     * @brief Print readable data of matrix B
     */
    void PrintMatrixB();

    /**
     * @brief Print readable data of matrix C
     */
    void PrintMatrixC();

    /**
     * @brief copy input
     */
    bool CopyInput();

    /**
     * @brief copy output
     */
    bool CopyOutput();

private:
    uint8_t *devMatrixA_ = nullptr;
    uint8_t *devMatrixB_ = nullptr;
    uint8_t *devMatrixC_ = nullptr;

    uint8_t *hostMatrixA_ = nullptr;
    uint8_t *hostMatrixB_ = nullptr;
    uint8_t *hostMatrixC_ = nullptr;

    uint32_t m_;
    uint32_t n_;
    uint32_t k_;

    uint8_t *devAlpha_ = nullptr;
    uint8_t *devBeta_ = nullptr;
    uint8_t hostAlpha_[8] = {0};
    uint8_t hostBeta_[8] = {0};

    size_t sizeA_;
    size_t sizeB_;
    size_t sizeC_;
    size_t sizeAlphaBeta_;

    aclDataType inputType_;
    aclDataType outputType_;
};

#endif // GEMM_RUNNER_H
