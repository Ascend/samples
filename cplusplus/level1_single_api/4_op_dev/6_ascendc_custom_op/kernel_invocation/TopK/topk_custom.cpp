/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 */
#include "topk_custom.h"
using namespace tik2;

template <typename T>
__aicore__ inline void KernelTopk<T>::Init(GM_ADDR x, GM_ADDR y, GM_ADDR workspace, GM_ADDR sync,
    GM_ADDR param) {
    x_gm.SetGlobalBuffer(reinterpret_cast<__gm__ T *>(x));
    y_gm.SetGlobalBuffer(reinterpret_cast<__gm__ T *>(y));
    workspace_gm.SetGlobalBuffer(reinterpret_cast<__gm__ T *>(workspace));
    sync_gm.SetGlobalBuffer(reinterpret_cast<__gm__ int32_t *>(sync));
    param_gm.SetGlobalBuffer(reinterpret_cast<__gm__ int32_t *>(param));

    tpipe.InitBuffer(vecIn, 1, HALF_UB_SIZE);
    tpipe.InitBuffer(vecOut, 1, HALF_UB_SIZE);

    // get k/total_num/isAscend value
    auto x_buf = vecIn.AllocTensor<int32_t>();
    DataCopy(x_buf, param_gm, 8);
    pipe_barrier(PIPE_ALL);
    int k = x_buf.GetValue(0);
    int totalNum = x_buf.GetValue(1);
    int isAscendVal = x_buf.GetValue(2);
    vecIn.FreeTensor(x_buf);

    kNum = k;
    isAscend = isAscendVal;
    // total_num must be scale of 8192 for fp16, scale of 4096 for fp32
    total_num = totalNum;

    if (IsSameType<T, half>::value) {
        ub_factor = 4096;
        DEFAULT_REPEAT = 128;
    } else if (IsSameType<T, float>::value) {
        ub_factor = 2048;
        DEFAULT_REPEAT = 64;
    }

    block_factor = 2 * ub_factor;
    if (total_num > block_factor) {
        int tmp = total_num / max_core_num;
        if (tmp <= block_factor) {
            core_num = total_num / block_factor;
        } else {
            block_factor = tmp;
            core_num = max_core_num;
        }
    }

    last_core = kNum / block_factor;
    remainK = kNum % block_factor;
    if (remainK == 0) {
        last_core -= 1;
        remainK = block_factor;
    }
    proposal_ub_factor = ub_factor * 8;
    loop_num = total_num / ub_factor;
    loop_num = (loop_num + 2 - 1) / 2 * 2;
    for (size_t i = 0; i < MERGE_4; ++i) {
        element_lengths[i] = static_cast<uint16_t>(ub_factor / 2);
    }
    mrg_params = {element_lengths, false, 0b1111, 1};
    DEFAULT_DATA_SIZE = DEFAULT_REPEAT * 16;
    DEFAULT_PROPOSAL_SIZE = DEFAULT_DATA_SIZE * 8;
}

template <typename T>
__aicore__ inline void KernelTopk< T>::CopyIn() {
    auto x_buf = vecIn.AllocTensor<T>();
    DataCopy(x_buf, x_gm[cur_in], ub_factor);
    vecIn.EnQue(x_buf);
    cur_in += ub_factor;
}

template <typename T>
__aicore__ inline void KernelTopk<T>::SortInUb() {
    auto x_buf = vecIn.DeQue<T>();
    auto y_buf = vecOut.AllocTensor<T>();
    auto tmp_buf = y_buf[proposal_ub_factor];

    if (isAscend == 1) {
        Muls<T>(x_buf, x_buf, -1.0, ub_factor);
    }
    pipe_barrier(PIPE_V);
    Concat(tmp_buf, x_buf, DEFAULT_REPEAT, IDX);
    Concat(tmp_buf[DEFAULT_PROPOSAL_SIZE], x_buf[DEFAULT_DATA_SIZE], DEFAULT_REPEAT, IDX);
    pipe_barrier(PIPE_V);
    RpSort16(y_buf, tmp_buf, DEFAULT_REPEAT);
    RpSort16(y_buf[DEFAULT_PROPOSAL_SIZE], tmp_buf[DEFAULT_PROPOSAL_SIZE], DEFAULT_REPEAT);

    vecIn.FreeTensor(x_buf);
    LocalTensor<T> buf_list[2] = {y_buf, tmp_buf};
    uint16_t size = MIN_SORT_SIZE;
    for (int i = 0; i < MERGE_4; i++) {
        auto src = buf_list[i % 2];
        auto dst = buf_list[(i + 1) % 2];
        uint16_t element_lengths[4] = {size, size, size, size};
        MrgSort4Info params{element_lengths, false, 0b1111, static_cast<uint16_t>(ub_factor / size / MERGE_4)};
        if (i == (MERGE_4 - 1) && ub_factor == 2048) {
            params = {element_lengths, false, 0b0011, static_cast<uint16_t>(1)};
        }
        pipe_barrier(PIPE_V);
        MrgSort4(
            dst, {src, src[size * PRO_SIZE], src[size * PRO_SIZE * 2], src[size * PRO_SIZE * 3]}, params);
        size *= MERGE_4;  // 16->64->256->1024
    }
    vecOut.EnQue(y_buf);
}

template <typename T>
__aicore__ inline void KernelTopk<T>::GmToUb() {
    DoWait();
    auto x_buf = vecIn.AllocTensor<T>();
    DataCopy(x_buf, workspace_gm[cur_from_gm], proposal_ub_factor * 2);
    vecIn.EnQue(x_buf);
    cur_from_gm += proposal_ub_factor * 2;
}

template <typename T>
__aicore__ inline void KernelTopk<T>::OddEvenSort() {
    auto y_buf = vecOut.AllocTensor<T>();
    auto x_buf = vecIn.DeQue<T>();
    MrgSort4(y_buf,
        {x_buf, x_buf[DEFAULT_PROPOSAL_SIZE], x_buf[DEFAULT_PROPOSAL_SIZE * 2], x_buf[DEFAULT_PROPOSAL_SIZE * 3]},
        mrg_params);
    if (stage >= loop_num) {
        pipe_barrier(PIPE_V);
        for (int i = 0; i < ub_factor * 2 / DEFAULT_DATA_SIZE; i++) {
            Extract(y_buf[i * DEFAULT_DATA_SIZE], y_buf[i * DEFAULT_PROPOSAL_SIZE], DEFAULT_REPEAT, IDX);
        }
        if (isAscend == 1) {
            pipe_barrier(PIPE_V);
            Muls<T>(y_buf, y_buf, -1.0, ub_factor * 2);
        }
    }
    vecOut.EnQue<T>(y_buf);
    vecIn.FreeTensor(x_buf);
}

template <typename T>
__aicore__ inline void KernelTopk<T>::CopyOut() {
    if (stage < loop_num) {
        auto y_buf = vecOut.DeQue<T>();
        auto buffer_len = stage < 0 ? proposal_ub_factor : proposal_ub_factor * 2;
        DataCopy(workspace_gm[cur_to_gm], y_buf, buffer_len);
        vecOut.FreeTensor(y_buf);
        cur_to_gm += buffer_len;
        DoSet();
    } else {
        auto y_buf = vecOut.DeQue<T>();
        if (block_idx == last_core) {
            if (remainK > ub_factor * 2) {
                DataCopy(y_gm[cur_out], y_buf, ub_factor * 2);
                remainK -= ub_factor * 2;
            } else if (remainK > 0) {
                DataCopy(y_gm[cur_out], y_buf, remainK);
                remainK = 0;
            }
        } else if (block_idx < last_core) {
            DataCopy(y_gm[cur_out], y_buf, ub_factor * 2);
        }
        vecOut.FreeTensor(y_buf);
        cur_out += ub_factor * 2;
    }
}

template <typename T>
__aicore__ inline void KernelTopk<T>::DoWait() {
    auto sync_buf = vecIn.AllocTensor<int32_t>();
    if (stage % 2 == 0 && stage != 0 && block_idx > 0 && cur_from_gm == block_begin) {
        IBWait(sync_gm, sync_buf, block_idx - 1, 1);
    } else if (stage % 2 == 1 && block_idx < core_num - 1 && cur_from_gm >= block_end - proposal_ub_factor) {
        IBWait(sync_gm, sync_buf, block_idx + 1, 0);
    }
    vecIn.FreeTensor(sync_buf);
}

template <typename T>
__aicore__ inline void KernelTopk<T>::DoSet() {
    auto sync_buf = vecOut.AllocTensor<int32_t>();
    if (stage >= 0 && stage % 2 == 0 && block_idx > 0 && cur_to_gm == block_begin + proposal_ub_factor * 2) {
        IBSet(sync_gm, sync_buf, block_idx, 0);
    } else if (stage >= 0 && stage % 2 == 1 && block_idx < core_num - 1 && cur_to_gm >= block_end) {
        IBSet(sync_gm, sync_buf, block_idx, 1);
    }
    vecOut.FreeTensor(sync_buf);
}

template <typename T>
__aicore__ inline void KernelTopk<T>::DoSmallK(int scale) {
    {
        auto x_buf = vecIn.AllocTensor<T>();
        DataCopy(x_buf, workspace_gm[cur_from_gm], proposal_ub_factor * 2);
        vecIn.EnQue(x_buf);
        cur_from_gm += proposal_ub_factor * 2;
    }
    {
        auto y_buf = vecOut.AllocTensor<T>();
        auto x_buf = vecIn.DeQue<T>();
        if (scale == 2) {
            MrgSort4(y_buf,
                {x_buf, x_buf[DEFAULT_PROPOSAL_SIZE], x_buf[DEFAULT_PROPOSAL_SIZE * 2],
                x_buf[DEFAULT_PROPOSAL_SIZE * 3]}, mrg_params);
        } else {
            // copy data from x_buf to y_buf
            Muls<T>(y_buf, x_buf, 1.0, proposal_ub_factor * scale);
        }
        for (int i = 0; i < ub_factor * scale / DEFAULT_DATA_SIZE; i++) {
            Extract(y_buf[i * DEFAULT_DATA_SIZE], y_buf[i * DEFAULT_PROPOSAL_SIZE], DEFAULT_REPEAT, IDX);
        }
        if (isAscend == 1) {
            pipe_barrier(PIPE_V);
            Muls<T>(y_buf, y_buf, -1.0, ub_factor * scale);
        }
        vecOut.EnQue<T>(y_buf);
        vecIn.FreeTensor(x_buf);
    }
    {
        auto y_buf = vecOut.DeQue<T>();
        DataCopy(y_gm[cur_out], y_buf, kNum);
        vecOut.FreeTensor(y_buf);
        cur_out += ub_factor * scale;
    }
}

template <typename T>
__aicore__ inline void KernelTopk<T>::Process() {
    cur_in = block_idx * block_factor;
    cur_out = cur_in;
    block_begin = block_idx * block_factor * PRO_SIZE;
    block_end = (block_idx + 1) * block_factor * PRO_SIZE;
    stage = -1;
    cur_to_gm = block_begin;
    while (cur_to_gm < block_end) {
        CopyIn();
        SortInUb();
        CopyOut();
    }
    cur_from_gm = block_begin;
    cur_to_gm = cur_from_gm;
    pipe_barrier(PIPE_ALL);
    if (total_num <= 2 * ub_factor) {
        DoSmallK(total_num / ub_factor);
    } else {
        while (cur_out < (block_idx * block_factor + block_factor)) {
            if (stage < 0 || cur_to_gm >= block_end || cur_from_gm + 2 * proposal_ub_factor > total_num * PRO_SIZE) {
                stage++;
                cur_from_gm = stage % 2 == 0 ? block_begin : block_begin + proposal_ub_factor;
                cur_to_gm = cur_from_gm;
            }
            if (cur_from_gm + 2 * proposal_ub_factor > total_num * PRO_SIZE) {
                continue;
            }
            GmToUb();
            OddEvenSort();
            CopyOut();
            pipe_barrier(PIPE_ALL);
            if (remainK == 0) {
                break;
            }
        }
    }
}

extern "C" __global__ __aicore__ void topk_custom(GM_ADDR x, GM_ADDR y,
    GM_ADDR workspace, GM_ADDR sync, GM_ADDR param) {
    KernelTopk<half> op;
    op.Init(x, y, workspace, sync, param);
    op.Process();
}

#ifndef __CCE_KT_TEST__
// call of kernel function
void topk_custom_do(uint32_t blockDim, void* l2ctrl, void* stream,
    uint8_t* x, uint8_t* y, uint8_t* workspace, uint8_t* sync, uint8_t* param)
{
    topk_custom<<<blockDim, l2ctrl, stream>>>(x, y, workspace, sync, param);
}
#endif