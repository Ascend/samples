/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 */
#ifndef KERNEL_SORT_H
#define KERNEL_SORT_H

#include "kernel_operator.h"

namespace tik2 {
template <typename T>
class KernelTopk {
public:
    __aicore__ inline KernelTopk() = default;
    __aicore__ void Init(GM_ADDR x, GM_ADDR y, GM_ADDR workspace, GM_ADDR sync, GM_ADDR param);
    __aicore__ void Process();

private:
    __aicore__ inline void InitParams();
    __aicore__ inline void CopyIn();
    __aicore__ inline void GmToUb();
    __aicore__ inline void SortInUb();
    __aicore__ inline void OddEvenSort();
    __aicore__ inline void CopyOut();
    __aicore__ inline void CopyOutTemp();
    __aicore__ inline void DoSet();
    __aicore__ inline void DoWait();
    __aicore__ inline void DoSmallK(int scale);

private:
    GlobalTensor<T> x_gm;
    GlobalTensor<T> y_gm;
    GlobalTensor<T> workspace_gm;
    GlobalTensor<int32_t> sync_gm;
    GlobalTensor<int32_t> param_gm;

    TPipe tpipe;
    TQue<QuePosition::VECIN, 1> vecIn;
    TQue<QuePosition::VECOUT, 1> vecOut;

    int cur_in;
    int cur_out;
    int cur_to_gm;
    int cur_from_gm;
    int block_begin;
    int block_end;
    int stage;

    int kNum;
    int remainK;
    int last_core;
    int isAscend = 0;
    int core_num = 1;
    int total_num = core_num * 8 * 1024;
    int block_factor = total_num / core_num;
    int ub_factor = 4096;
    int proposal_ub_factor = ub_factor * 8;
    int factor_num = block_factor / ub_factor;
    int loop_num = total_num / ub_factor;
    uint16_t element_lengths[4] = {static_cast<uint16_t>(ub_factor / 2), static_cast<uint16_t>(ub_factor / 2),
        static_cast<uint16_t>(ub_factor / 2), static_cast<uint16_t>(ub_factor / 2)};
    MrgSort4Info mrg_params{element_lengths, false, 0b1111, 1};
    int DEFAULT_REPEAT = 128;
    int DEFAULT_DATA_SIZE = 128 * 16;
    int DEFAULT_PROPOSAL_SIZE = DEFAULT_DATA_SIZE * 8;

    const int max_core_num = 4;
    const int HALF_UB_SIZE = 128 * 1024;
    const int MIN_SORT_SIZE = 16;
    const int PRO_SIZE = 8;
    const int IDX = 4;
    const int MERGE_4 = 4;
};
}  // namespace tik2
#endif
