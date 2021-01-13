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
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <memory>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "atlas_utils.h"
#include "dvpp_mem_mgr.h"


using namespace std;

namespace {
    const uint32_t kDvppMemAlign = 128;
    const uint32_t kDefaultBlockNum = 2048;
    const uint32_t kLevelInvalid = kLevelNum;
    const uint32_t kPoolIdInvalid = kPerLevelPoolsNum;
    const uint32_t kLevelSmallMax = 512;
    const uint32_t kLevelLargeMax = 64;

    const uint32_t kLevel4k = 4 * 1024;
    const uint32_t kLevel16k = 16 * 1024;   
    const uint32_t kLevel64k = 64 * 1024;
    const uint32_t kLevel256k = 256 * 1024;
    const uint32_t kLevel512k = 512 * 1024;
    const uint32_t kLevel1024k = 1024 * 1024;
    const uint32_t kLevel2m = 2 * 1024 * 1024;
    const uint32_t kLevel4m = 4 * 1024 * 1024;
    const uint32_t kLevel8m = 8 * 1024 * 1024;
}


DvppMemPool::DvppMemPool() : 
blockNum_(0),
freeList_(kDefaultBlockNum) {
}

DvppMemPool::~DvppMemPool() {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    do {
        void* buffer = freeList_.Pop();
        if (!buffer) {
            break;
        } 
        acldvppFree(buffer);
        blockNum_--;
    }while(1);
}

void* DvppMemPool::MallocMem(uint32_t size, bool& isNewBlock) {
    isNewBlock = false;
    std::lock_guard<std::mutex> lock(mutex_lock_);
    void* buffer = freeList_.Pop();
    if (!buffer) {        
        aclError aclRet = acldvppMalloc(&buffer, size);
        if ((aclRet != ACL_ERROR_NONE) || (buffer == nullptr)) {
            ATLAS_LOG_ERROR("Acl dvpp malloc return error %d", aclRet);
            return nullptr;
        }
        blockNum_++;
        isNewBlock = true;
    } 

    usedList_.push_back(buffer);   

    return buffer;
}

void DvppMemPool::FreeMem(void* ptr) {
    std::lock_guard<std::mutex> lock(mutex_lock_);
    list<void*>::iterator it = usedList_.begin();
    while (it != usedList_.end()) {
        if (*it == ptr) {            
            usedList_.erase(it); 
            freeList_.Push(ptr);
            return;
        }
        it++;
    }

    ATLAS_LOG_ERROR("Free memory %p in pool failed", ptr);
    it = usedList_.begin();
    while (it != usedList_.end()) {
        it++;
    }

    return;
}

DvppMemPoolMgr::DvppMemPoolMgr() {
    levelList_[0].size = kLevel4k;
    levelList_[0].maxBlockNum = kLevelSmallMax;

    levelList_[1].size = kLevel16k;
    levelList_[1].maxBlockNum = kLevelSmallMax;

    levelList_[2].size = kLevel64k;
    levelList_[2].maxBlockNum = kLevelSmallMax;

    levelList_[3].size = kLevel256k;
    levelList_[3].maxBlockNum = kLevelSmallMax;

    levelList_[4].size = kLevel512k;
    levelList_[4].maxBlockNum = kLevelSmallMax;

    levelList_[5].size = kLevel1024k;
    levelList_[5].maxBlockNum = kLevelSmallMax;

    levelList_[6].size = kLevel2m;
    levelList_[6].maxBlockNum = kLevelLargeMax;

    levelList_[7].size = kLevel4m;
    levelList_[7].maxBlockNum = 512;

    levelList_[8].size = kLevel8m;
    levelList_[8].maxBlockNum = kLevelLargeMax;
}

void* DvppMemPoolMgr::MallocMem(uint32_t size) {
    uint32_t wrapSize = size + kDvppMemAlign;
    uint32_t level = ChooseLevel(wrapSize);
    if (level >= kLevelInvalid) {        
        return MallocDvppMemory(wrapSize);      
    }
    
   
    uint32_t index = ChoosePool(level);  
    if (index >= kPoolIdInvalid) {
        return MallocDvppMemory(wrapSize);   
    }  

    bool isNewBlock = false;
    void* buffer = levelList_[level].poolTbl[index].MallocMem(wrapSize, isNewBlock);
    if (!buffer) {
        return nullptr;
    }

    if (isNewBlock) {        
        levelList_[level].totalBlockNum++;
    }

    return WrapMem(buffer, level, index, MEM_POOL_MALLOC, size);
}

void* DvppMemPoolMgr::MallocDvppMemory(uint32_t size) {
    void* buffer = nullptr;
    aclError aclRet = acldvppMalloc(&buffer, size);   
    if ((aclRet != ACL_ERROR_NONE) || (buffer == nullptr)) {
        ATLAS_LOG_ERROR("Acl dvpp malloc return error %d", aclRet);
        return nullptr;
    } 
    return WrapMem(buffer, kLevelInvalid, 0, MEM_ACL_MALLOC, size);
}

void* DvppMemPoolMgr::WrapMem(void* buffer, uint32_t level, 
                              uint32_t poolId, MallocType mallocType, uint32_t size) {
    MallocInfo *info = (MallocInfo *)buffer;
    info->level = level;
    info->poolId = poolId;
    info->size = size;
    info->mallocType = mallocType;
    return (void *)((uint8_t*)buffer + kDvppMemAlign);    
}

void DvppMemPoolMgr::FreeMem(void* ptr) {  
    void* buffer = (void *)((uint8_t *)ptr - kDvppMemAlign);     
    MallocInfo *info = (MallocInfo *)((uint8_t *)ptr - kDvppMemAlign);
    if (info->mallocType == MEM_ACL_MALLOC) {
        acldvppFree(buffer);
        return;
    }

    DvppMemPool& pool = levelList_[info->level].poolTbl[info->poolId];
    pool.FreeMem(buffer);

    return;
}

uint32_t DvppMemPoolMgr::ChooseLevel(uint32_t size) {
    for (uint32_t i = 0; i < kLevelNum; i++) {
        if (size < levelList_[i].size) {
            return i;
        }
    }

    return kLevelInvalid;
}

uint32_t DvppMemPoolMgr::ChoosePool(uint32_t level) {
    uint32_t index = 0;
    PoolLevel& poolLevel = levelList_[level];
    if (poolLevel.totalBlockNum >= poolLevel.maxBlockNum) {
        uint32_t maxFreeNum = poolLevel.poolTbl[0].FreeBlockNum();
        for (uint32_t i = 1; i < kPerLevelPoolsNum; i++) {
            if (maxFreeNum < poolLevel.poolTbl[i].FreeBlockNum()) {
                maxFreeNum = poolLevel.poolTbl[i].FreeBlockNum();
                index = i;
            }
            if (maxFreeNum == 0) {
                index = kPoolIdInvalid;
            }
        }
    } else {
        uint32_t minBlockNum = poolLevel.poolTbl[0].BlockNum();
        for (uint32_t i = 1; i < kPerLevelPoolsNum; i++) {
            if (minBlockNum > poolLevel.poolTbl[i].BlockNum()) {
                minBlockNum = poolLevel.poolTbl[i].BlockNum();
                index = i;
            }
        }
    }

    return index;
}

void DvppMemPoolMgr::PrintPoolInfo() {
    printf("Mem pool: total malloc times %d, total free times %d\n",
           totalMallocTimes_, totalFreeTimes_);
    for (uint32_t i = 0; i < kLevelNum; i++) {
        if (levelList_[i].totalBlockNum == 0)
            continue;

        printf("Level: %d\n", levelList_[i].size);
        for (uint32_t j = 0; j < kPerLevelPoolsNum; j++) {
            DvppMemPool& pool = levelList_[i].poolTbl[j];
            if (pool.BlockNum() == 0) 
                continue;

            printf("pool %d: block num %d, free:%d, used: %d\n", 
                   j, pool.BlockNum(), pool.FreeBlockNum(), 
                   pool.UsedBlockNum());
        }
    }
}


void* AtlasDvppMalloc(uint32_t size) {
    DvppMemPoolMgr& inst = DvppMemPoolMgr::GetInstance();
    inst.StatisticMalloc();
    return inst.MallocMem(size);
}

void AtlasDvppFree(void* ptr) {
    DvppMemPoolMgr& inst = DvppMemPoolMgr::GetInstance();
    inst.StatisticFree();
    return inst.FreeMem(ptr);
}

void PrintDvppMgrInfo() {
    DvppMemPoolMgr& inst = DvppMemPoolMgr::GetInstance();
    inst.PrintPoolInfo();
}
 
    
