/**
* @file fv_search.cpp
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "fv_search.h"
#include "string.h"
#include "common.h"
#include "fv_resource.h"

#define RAND_SEED_VAL(id1) (1000 * (id1))

bool FVSearchBase::FeatureAccurateDelModify(bool isDel, uint32_t id0, uint32_t id1, uint32_t offset,
    const uint8_t *featureData, size_t dataLen)
{
    // 1.create buff and copy data
    void *inputData = nullptr;
    ACL_REQUIRES_SUCCESS(aclrtMalloc(&inputData, BASESHORT_D_LEN, ACL_MEM_MALLOC_NORMAL_ONLY),
        ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    std::shared_ptr<void> inputDataPtr(inputData, [](void *p){(void)aclrtFree(p);});
    aclrtMemcpyKind kind = ACL_MEMCPY_DEVICE_TO_DEVICE;
    if (!FVResource::GetInstance().IsDevice()) {
        kind = ACL_MEMCPY_HOST_TO_DEVICE;
    }
    ACL_REQUIRES_SUCCESS(aclrtMemcpy(inputData, BASESHORT_D_LEN, featureData, dataLen, kind),
        ERROR_LOG("aclrtMemcpy failed!!!"), return false);

    // 2.Create Feature Info
    auto featureInfo = aclfvCreateFeatureInfo(id0, id1, offset, BASESHORT_D_LEN, 1,
                                              reinterpret_cast<uint8_t *>(inputData), BASESHORT_D_LEN);
    if (featureInfo == nullptr) {
        ERROR_LOG("aclfvCreateFeatureInfo failed!!!");
        return false;
    }
    std::shared_ptr<aclfvFeatureInfo> featureInfoPtr(featureInfo,
                                                     [](aclfvFeatureInfo *p){(void)aclfvDestroyFeatureInfo(p);});

    // 3.Delete or Modify
    if (isDel) {
        ACL_REQUIRES_SUCCESS(aclfvDel(featureInfo), ERROR_LOG("aclfvDel failed!!!"), return false);
        INFO_LOG("Accurate Delete success!!!");
    } else {
        ACL_REQUIRES_SUCCESS(aclfvModify(featureInfo), ERROR_LOG("aclfvModify failed!!!"), return false);
        INFO_LOG("Accurate Modify success!!!");
    }
    return true;
}

RunFVSearch::~RunFVSearch()
{
    (void)Finalize();
}

bool RunFVSearch::Initialize(aclfvSearchType type)
{
    if (type != SEARCH_1_N) {
        ERROR_LOG("this sample only support SEARCH_1_N.");
        return false;
    }
    fvSearch_ = new FVSearch1N();
    if (fvSearch_ == nullptr) {
        ERROR_LOG("alloc for FVSearch1N failed.");
        return false;
    }
    size_t fsNum = 100000;
    int32_t deviceId = 0;
    return FVResource::GetInstance().Initialize(fsNum, deviceId);
}

bool RunFVSearch::Add()
{
    if (fvSearch_ == nullptr) {
        ERROR_LOG("Add failed, Initialize is required first.");
        return false;
    }

    // 1. add first features
    uint32_t offset = 0;
    uint32_t featureCount = 1000;
    auto ret = fvSearch_->FeatureAdd(0, 0, offset, featureCount);
    if (!ret) {
        return false;
    }

    // 2. add features into the same library
    offset += featureCount; // offset in the same library must be continuous!!!
    ret = fvSearch_->FeatureAdd(0, 0, offset, featureCount);
    if (!ret) {
        return false;
    }

    uint8_t featureData[BASESHORT_D_LEN];
    for (size_t i = 0; i < BASESHORT_D_LEN; i++) {
        featureData[i] = static_cast<uint8_t>(i);
    }
    (void)fvSearch_->FeatureAccurateDelModify(true, 0, 0, 1338, featureData, sizeof(featureData));

    // 3. add features into another library
    offset = 0; // different library has different offset
    return fvSearch_->FeatureAdd(1, 1, offset, featureCount);
}

bool RunFVSearch::Search()
{
    if (fvSearch_ == nullptr) {
        ERROR_LOG("Search failed, Initialize is required first.");
        return false;
    }

    uint32_t queryCnt = 1; // for 1:N, query count must be 1
    uint32_t topK = 5; // top result
    return fvSearch_->FeatureSearch(queryCnt, topK);
}

bool RunFVSearch::Delete()
{
    if (fvSearch_ == nullptr) {
        return false;
    }
    // repo delete
    uint32_t id0Min = 0;
    uint32_t id0Max = 1023;
    uint32_t id1Min = 0;
    uint32_t id2Max = 1023;
    return fvSearch_->FeatureRepoDel(id0Min, id0Max, id1Min, id2Max);
}

void  RunFVSearch::Finalize()
{
    (void)Delete();
    if (fvSearch_ != nullptr) {
        delete fvSearch_;
        fvSearch_ = nullptr;
    }
    FVResource::GetInstance().Finalize();
}

bool FVSearch1N::FeatureAdd(uint32_t id0, uint32_t id1, uint32_t offset, uint32_t featureCount)
{
    INFO_LOG("FeatureAdd: featureCount is %u", featureCount);
    // 0. generate data
    uint32_t featureLen = BASESHORT_D_LEN;
    void *featureData = BaseShortFeaAlloc(RAND_SEED_VAL(1), static_cast<size_t>(featureCount), 0);
    if (featureData == nullptr) {
        ERROR_LOG("BaseShortFeaAlloc failed!!!");
        return false;
    }
    std::shared_ptr<void> feaBufPtr(featureData, [](void *p) {(void)aclrtFreeHost(p);});
    void *inputData = featureData;
    std::shared_ptr<void> inputDataPtr = nullptr;
    aclError ret;
    if (!FVResource::GetInstance().IsDevice()) {
        // copy data to device
        ret = aclrtMalloc(&inputData, featureLen * featureCount, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMalloc failed, result = %d", ret);
            return false;
        }
        inputDataPtr.reset(inputData, [](void *p) {(void)aclrtFree(p);});
        ret = aclrtMemcpy(inputData,
                          featureLen * featureCount,
                          featureData,
                          featureLen * featureCount,
                          ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMemcpy failed, result = %d", ret);
            return false;
        }
    }

    // 1. create acl feature info
    auto featureInfo = aclfvCreateFeatureInfo(
            id0, id1, offset, featureLen, featureCount, reinterpret_cast<uint8_t *>(inputData), featureLen * featureCount);
    if (featureInfo == nullptr) {
        ERROR_LOG("aclfvCreateFeatureInfo failed, result = %d", ret);
        return false;
    }
    INFO_LOG("aclfvCreateFeatureInfo success!!!");

    // 2. repo add
    ret = aclfvRepoAdd(SEARCH_1_N, featureInfo);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclfvRepoAdd failed, result = %d", ret);
        aclfvDestroyFeatureInfo(featureInfo);
        return false;
    }
    INFO_LOG("aclfvRepoAdd success!!!");

    // 3. destroy acl feature info
    aclfvDestroyFeatureInfo(featureInfo);
    return true;
}

bool FVSearch1N::FeatureSearchPreProc(uint32_t queryCnt, uint32_t topK)
{
    uint32_t dataLen = queryCnt * topK * sizeof(uint32_t);
    uint32_t resultNumDataLen = queryCnt * sizeof(uint32_t);

    const uint32_t tableLen = 32 * 1024; // 32KB for one
    uint32_t tableDataLen = queryCnt * tableLen;

    // 1. generate table data
    uint8_t *tableDataTmp = (uint8_t *)AdcTabInit(RAND_SEED_VAL(1), queryCnt * 1024);
    if (tableDataTmp == nullptr) {
        ERROR_LOG("AdcTableInit failed!!!");
        return false;
    }
    std::shared_ptr<void> tableDataTmpPtr(tableDataTmp, [](void *p) {(void)aclrtFreeHost(p);});

    // 2. alloc device memory for table data
    void *devPtr = nullptr;
    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, tableDataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    tableDataDev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});

    // 3. copy table data to device memory
    uint8_t *devPtrTmp = reinterpret_cast<uint8_t *>(devPtr);
    for (uint32_t i = 0; i < queryCnt; ++i) {
        for (uint32_t j = 0; j < 32; ++j) {
            uint8_t *dst = devPtrTmp + i * 32 * 1024 + j * 1024;
            uint8_t *src = tableDataTmp + i * 1024;
            ACL_REQUIRES_SUCCESS(aclrtMemcpy(dst, 1024, src, 1024, ACL_MEMCPY_HOST_TO_DEVICE),
                                 ERROR_LOG("aclrtMemcpy failed!!!"), return false);
        }
    }

    // 4. alloc other device memory
    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, resultNumDataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    resultNumDev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});

    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, dataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    id0Dev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});

    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, dataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    id1Dev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});

    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, dataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    resultOffsetDev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});

    ACL_REQUIRES_SUCCESS(aclrtMalloc(&devPtr, dataLen, ACL_MEM_MALLOC_NORMAL_ONLY),
                         ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    resultDistanceDev_.reset(devPtr, [](void *p) {(void)aclrtFree(p);});  // sizeof(float) == sizeof(uint32_t)

    // 5. create query resource
    aclfvQueryTable *searchQueryTable =
            aclfvCreateQueryTable(queryCnt, tableLen, reinterpret_cast<uint8_t *>(tableDataDev_.get()), tableDataLen);
    searchQueryTable_.reset(searchQueryTable, [](aclfvQueryTable *p){(void)aclfvDestroyQueryTable(p);});

    aclfvRepoRange *searchRange = aclfvCreateRepoRange(0, 1023, 0, 1023); // id0[0, 1024), id1[0, 1024)
    searchRange_.reset(searchRange, [](aclfvRepoRange *p){(void)aclfvDestroyRepoRange(p);});

    aclfvSearchInput *searchInput = aclfvCreateSearchInput(searchQueryTable, searchRange, topK);
    searchInput_.reset(searchInput, [](aclfvSearchInput *p){(void)aclfvDestroySearchInput(p);});

    aclfvSearchResult *searchResult = aclfvCreateSearchResult(queryCnt,
                                                              reinterpret_cast<uint32_t *>(resultNumDev_.get()),
                                                              resultNumDataLen,
                                                              reinterpret_cast<uint32_t *>(id0Dev_.get()),
                                                              reinterpret_cast<uint32_t *>(id1Dev_.get()),
                                                              reinterpret_cast<uint32_t *>(resultOffsetDev_.get()),
                                                              reinterpret_cast<float *>(resultDistanceDev_.get()),
                                                              dataLen);
    searchResult_.reset(searchResult, [](aclfvSearchResult *p){(void)aclfvDestroySearchResult(p);});
    return true;
}

bool FVSearch1N::FeatureSearch(uint32_t queryCnt, uint32_t topK)
{
    // 1. search prepare process
    INFO_LOG("feature search prepare processing.");
    if (!FeatureSearchPreProc(queryCnt, topK)) {
        ERROR_LOG("FeatureSearchPreProc failed!!!");
        return false;
    }

    // 2. search
    INFO_LOG("feature search running.");
    ACL_REQUIRES_SUCCESS(aclfvSearch(SEARCH_1_N, searchInput_.get(), searchResult_.get()),
                         ERROR_LOG("aclfvSearch failed!!!"), return false);

    // 3. post process
    INFO_LOG("feature search post processing.");
    if (!FeatureSearchPostProc(queryCnt, topK)) {
        ERROR_LOG("FeatureSearchPreProc failed!!!");
        return false;
    }

    INFO_LOG("feature search success!!!");
    return true;
}

bool FVSearch1N::FeatureSearchPostProc(uint32_t queryCnt, uint32_t topK)
{
    // 1. get result data
    uint32_t dataLen = queryCnt * topK * sizeof(uint32_t);
    uint32_t *id0 = (uint32_t *)id0Dev_.get();
    uint32_t *id1 = (uint32_t *)id1Dev_.get();
    uint32_t *resultOffset= (uint32_t *)resultOffsetDev_.get();
    float *resultDistance = (float *)resultDistanceDev_.get();
    // use unique_ptr for auto release
    std::unique_ptr<uint32_t> id0Ptr = nullptr;
    std::unique_ptr<uint32_t> id1Ptr = nullptr;
    std::unique_ptr<uint32_t> resultOffsetPtr = nullptr;
    std::unique_ptr<float> resultDistancePtr = nullptr;
    if (!FVResource::GetInstance().IsDevice()) {
        // copy device data to host
        id0 = (uint32_t *)malloc(dataLen);
        id0Ptr.reset(id0);
        id1 = (uint32_t *)malloc(dataLen);
        id1Ptr.reset(id1);
        resultOffset = (uint32_t *)malloc(dataLen);
        resultOffsetPtr.reset(resultOffset);
        resultDistance = (float *)malloc(dataLen);
        resultDistancePtr.reset(resultDistance);

        ACL_REQUIRES_SUCCESS(aclrtMemcpy(id0, dataLen, id0Dev_.get(), dataLen, ACL_MEMCPY_DEVICE_TO_HOST),
                ERROR_LOG("aclrtMemcpy failed!!!"), return false);
        ACL_REQUIRES_SUCCESS(aclrtMemcpy(id1, dataLen, id0Dev_.get(), dataLen, ACL_MEMCPY_DEVICE_TO_HOST),
                ERROR_LOG("aclrtMemcpy failed!!!"), return false);
        ACL_REQUIRES_SUCCESS(
                aclrtMemcpy(resultOffset, dataLen, resultOffsetDev_.get(), dataLen, ACL_MEMCPY_DEVICE_TO_HOST),
                ERROR_LOG("aclrtMemcpy failed!!!"), return false);
        ACL_REQUIRES_SUCCESS(
                aclrtMemcpy(resultDistance, dataLen, resultDistanceDev_.get(), dataLen, ACL_MEMCPY_DEVICE_TO_HOST),
                ERROR_LOG("aclrtMemcpy failed!!!"), return false);
    }

    // 2. show result
    INFO_LOG("Search Result:");
    INFO_LOG("============================================================================");
    for (uint32_t i = 0; i < queryCnt; i++) {
        for (uint32_t j = 0; j < topK; ++j) {
            uint32_t i0 = id0[i * topK + j];
            uint32_t i1 = id1[i * topK + j];
            uint32_t offset = resultOffset[i * topK + j];
            float distance = resultDistance[i * topK + j];
            INFO_LOG("Search Index[%u] top[%u]: id0 = %u, id1 = %u, offset = %u, distance = %.2f",
                     i, j, i0, i1, offset, distance);
        }
    }
    INFO_LOG("============================================================================");
    return true;
}

bool FVSearch1N::FeatureRepoDel(uint32_t id0Min, uint32_t id0Max, uint32_t id1Min, uint32_t id1Max)
{
    aclfvRepoRange *repoRange = aclfvCreateRepoRange(id0Min, id0Max, id1Min, id1Max);
    if (repoRange == nullptr) {
        ERROR_LOG("aclfvCreateRepoRange failed!!!");
        return false;
    }
    std::shared_ptr<aclfvRepoRange> repoRangePtr(repoRange, [](aclfvRepoRange *p){(void)aclfvDestroyRepoRange(p);});

    ACL_REQUIRES_SUCCESS(aclfvRepoDel(SEARCH_1_N, repoRange),
            ERROR_LOG("aclfvRepoDel failed!!!"), return false);

    INFO_LOG("Repo delete success!!!");
    return false;
}