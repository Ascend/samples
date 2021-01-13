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
#pragma once

#include<list>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#include "thread_safe_queue.h"
#include "atlas_error.h"
#include "dvpp_mem_mgr.h"

namespace {
    const uint32_t kLevelNum = 9;
    const uint32_t kPerLevelPoolsNum = 4;
}

enum MallocType {
    MEM_ACL_MALLOC = 0,
    MEM_POOL_MALLOC,
    UNKNOW_MALLOC_TYPE,
};

struct MallocInfo {
    uint32_t level;
    uint32_t poolId;
    uint32_t size;
    MallocType mallocType;
};

class DvppMemPool {
public:
    DvppMemPool();
    ~DvppMemPool();

    void* MallocMem(uint32_t size, bool& isNewBlock);
    void FreeMem(void* ptr);

    uint32_t BlockNum() { return blockNum_; }
    uint32_t FreeBlockNum() { return freeList_.Size(); }
    uint32_t UsedBlockNum() { return usedList_.size(); }
private:  
    uint32_t blockNum_; 
    ThreadSafeQueue<void*> freeList_;
    std::list<void*> usedList_;
    mutable std::mutex mutex_lock_;
};

struct PoolLevel {
    uint32_t size;
    uint32_t maxBlockNum; 
    uint32_t totalBlockNum = 0;
    DvppMemPool poolTbl[kPerLevelPoolsNum];  
};

class DvppMemPoolMgr {
public:
    DvppMemPoolMgr();

	DvppMemPoolMgr(const DvppMemPoolMgr&) = delete;
	DvppMemPoolMgr& operator=(const DvppMemPoolMgr&) = delete;

    static DvppMemPoolMgr& GetInstance() {
		static DvppMemPoolMgr instance;
		return instance;
	}

    ~DvppMemPoolMgr() {};

    void* MallocMem(uint32_t size);
    void FreeMem(void* ptr);

    void StatisticMalloc() { totalMallocTimes_++; }
    void StatisticFree() { totalFreeTimes_++; }
    void PrintPoolInfo();

private:
    uint32_t ChooseLevel(uint32_t size);
    uint32_t ChoosePool(uint32_t level);    
    void* MallocDvppMemory(uint32_t size);
    void* WrapMem(void* buffer, uint32_t level, 
                  uint32_t poolId, MallocType mallocType, uint32_t size);

private:
    PoolLevel levelList_[kLevelNum]; 

    uint32_t totalMallocTimes_;
    uint32_t totalFreeTimes_; 
};

void* AtlasDvppMalloc(uint32_t size);
void AtlasDvppFree(void* ptr);
void PrintDvppMgrInfo();

