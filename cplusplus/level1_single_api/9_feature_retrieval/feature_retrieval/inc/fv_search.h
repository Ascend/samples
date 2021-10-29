/**
* @file fv_search.h
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef FV_SEARCH_H
#define FV_SEARCH_H
#include <iostream>
#include <memory>
#include "acl/ops/acl_fv.h"

class FVSearchBase {
public:
    FVSearchBase() = default;
    virtual ~FVSearchBase() = default;
    virtual bool FeatureAdd(uint32_t id0, uint32_t id1, uint32_t offset, uint32_t featureCount) = 0;
    virtual bool FeatureSearch(uint32_t queryCnt, uint32_t topK) = 0;
    virtual bool FeatureRepoDel(uint32_t id0Min, uint32_t id0Max, uint32_t id1Min, uint32_t id1Max) = 0;
    bool FeatureAccurateDelModify(bool isDel, uint32_t id0, uint32_t id1, uint32_t offset,
        const uint8_t *featureData, size_t dataLen);
};

// for 1:N
class FVSearch1N : public FVSearchBase {
public:
    FVSearch1N() = default;
    ~FVSearch1N() = default;
    bool FeatureAdd(uint32_t id0, uint32_t id1, uint32_t offset, uint32_t featureCount) override;
    bool FeatureSearch(uint32_t queryCnt, uint32_t topK) override;
    bool FeatureRepoDel(uint32_t id0Min, uint32_t id0Max, uint32_t id1Min, uint32_t id1Max) override;

private:
    bool FeatureSearchPreProc(uint32_t queryCnt, uint32_t topK);
    bool FeatureSearchPostProc(uint32_t queryCnt, uint32_t topK);

private:
    std::shared_ptr<void> tableDataDev_ = nullptr;
    std::shared_ptr<void> resultNumDev_ = nullptr;
    std::shared_ptr<void> id0Dev_ = nullptr;
    std::shared_ptr<void> id1Dev_ = nullptr;
    std::shared_ptr<void> resultOffsetDev_ = nullptr;
    std::shared_ptr<void> resultDistanceDev_ = nullptr;
    std::shared_ptr<aclfvQueryTable> searchQueryTable_ = nullptr;
    std::shared_ptr<aclfvRepoRange> searchRange_ = nullptr;
    std::shared_ptr<aclfvSearchInput> searchInput_ = nullptr;
    std::shared_ptr<aclfvSearchResult> searchResult_ = nullptr;
};

class RunFVSearch {
public:
    RunFVSearch() = default;
    ~RunFVSearch();
    bool Initialize(aclfvSearchType type);
    void Finalize();
    bool Add();
    bool Search();
    bool Delete();
private:
    FVSearchBase *fvSearch_ = nullptr;
};
#endif //FV_SEARCH_H
