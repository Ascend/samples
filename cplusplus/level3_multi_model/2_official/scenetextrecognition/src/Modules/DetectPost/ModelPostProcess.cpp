/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
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

#include "ModelPostProcess.h"

#include <string>
#include <vector>
#include <fstream>

#include "Log/Log.h"
#include "FastMath.h"
#include "Common/CommonType.h"

using namespace std;

namespace {
    const int BOX_STRIDE = 7;
    const float PIXEL_THRESHOLD = 0.9;

    bool IsNotDisjoint(std::vector<std::pair<int, int>> vec1, std::vector<std::pair<int, int>> vec2);
    bool ShouldMerge(const std::vector<std::pair<int, int>>& region, std::pair<int, int> activationPixel);
    std::vector<std::pair<int, int>> RegionNeighbor(std::vector<std::pair<int, int>> regionSet);
    void RegionGroup(std::vector<std::vector<std::pair<int, int>>> regionList,
                     std::vector<std::vector<int>> &groupVec);
    std::vector<int> RecRegionMerge(std::vector<std::vector<std::pair<int, int>>> regionList, int m,
                                    std::vector<int> &s);
    void ComputeBoxSize(uint32_t modelWidth, uint32_t modelHeight, uint32_t &boxWidth, uint32_t &boxHeight);
    void GeneratePixelsInfo(std::vector<std::shared_ptr<void>> outputDatas,
                            std::vector<std::vector<std::vector<float>>> &pixelsInfo,
                            std::vector<std::pair<int, int>> &activationPixels,
                            uint32_t boxWidth, uint32_t boxHeight);
    void GenerateRegionList(std::vector<std::pair<int, int>> activationPixels,
                            std::vector<std::vector<std::pair<int, int>>> &regionList);
    void GetBoxCoordinates(std::pair<int, int> pixelIndex, float score,
                           const std::vector<std::vector<std::vector<float>>> &pixelsInfo,
                           std::vector<int> &quadList, std::vector<float> &totalScore);


    /*
    * @description: Determine whether two vectors intersect
    * @param vec1
    * @param vec2
    * @return true if intersect else false
    */
    bool IsNotDisjoint(std::vector<std::pair<int, int>> vec1, std::vector<std::pair<int, int>> vec2)
    {
        for (size_t i = 0; i < vec2.size(); i++) {
            if (std::find(vec1.begin(), vec1.end(), vec2[i]) != vec1.end()) {
                return true;
            }
        }
        return false;
    }

    /*
    * @description: Determine whether two vectors intersect
    * @param vec1
    * @param vec2
    * @return true if intersect else false
    */
    bool ShouldMerge(const std::vector<std::pair<int, int>>& region, std::pair<int, int> activationPixel)
    {
        std::vector<std::pair<int, int>> tmp;
        tmp.push_back(std::make_pair(activationPixel.first, activationPixel.second - 1));
        bool res = IsNotDisjoint(region, tmp);
        return res;
    }

    std::vector<std::pair<int, int>> RegionNeighbor(std::vector<std::pair<int, int>> regionSet)
    {
        int minY = regionSet[0].second;
        int maxY = regionSet[0].second;
        int minX = regionSet[0].first;
        for (size_t i = 0; i < regionSet.size(); i++) {
            if (regionSet[i].second < minY) {
                minY = regionSet[i].second;
            }
            if (regionSet[i].second > maxY) {
                maxY = regionSet[i].second;
            }
            if (regionSet[i].first < minX) {
                minX = regionSet[i].first;
            }
        }
        for (size_t j = 0; j < regionSet.size(); j++) {
            regionSet[j].first += 1;
        }
        regionSet.push_back(std::make_pair(minX, minY));
        regionSet.push_back(std::make_pair(minX, maxY));
        return regionSet;
    }

    void RegionGroup(std::vector<std::vector<std::pair<int, int>>> regionList, std::vector<std::vector<int>> &groupVec)
    {
        int lenList = regionList.size();
        std::vector<int> s;
        for (int i = 0; i < lenList; i++) {
            s.push_back(i);
        }

        while (s.size() > 0) {
            int m = s[0];
            s.erase(s.begin());
            if (s.size() == 0) {
                groupVec.push_back({m});
            } else {
                std::vector<int> bb = RecRegionMerge(regionList, m, s);
                groupVec.push_back(bb);
            }
        }
    }

    std::vector<int> RecRegionMerge(std::vector<std::vector<std::pair<int, int>>> regionList, int m,
                                    std::vector<int> &s)
    {
        std::vector<int> rows = {m};
        std::vector<int> tmp;
        for (auto const &n: s) {
            if (IsNotDisjoint(RegionNeighbor(regionList[m]), regionList[n]) ||
                IsNotDisjoint(RegionNeighbor(regionList[n]), regionList[m])) {
                    tmp.push_back(n);
                }
        }
        for (auto const &d: tmp) {
            s.erase(remove(s.begin(), s.end(), d), s.end());
        }
        for (auto const &e: tmp) {
            std::vector<int> res = RecRegionMerge(regionList, e, s);
            rows.insert(rows.end(), res.begin(), res.end());
        }
        return rows;
    }

    /*
    * @description: Compute the Output box size
    * @param modelWidth  Model input width
    * @param modelHeight  Model input height
    * @param boxWidth  Output box width
    * @param boxHeight  Output box height
    */
    void ComputeBoxSize(uint32_t modelWidth, uint32_t modelHeight, uint32_t &boxWidth, uint32_t &boxHeight)
    {
        const int pixelSize = 4;
        boxWidth = modelWidth / pixelSize;
        boxHeight = modelHeight / pixelSize;
    }

    /*
    * @description: Generate the pixels info
    * @param outputDatas  Model output data
    * @param pixelsInfo  Pixels info with score map, vertex code and geo info
    * @param boxWidth  Output box width
    * @param boxHeight  Output box height
    */
    void GeneratePixelsInfo(std::vector<std::shared_ptr<void>> outputDatas,
                            std::vector<std::vector<std::vector<float>>> &pixelsInfo,
                            std::vector<std::pair<int, int>> &activationPixels, uint32_t boxWidth, uint32_t boxHeight)
    {
        int scoreStride = 3;
        float* outputData = static_cast<float *>(outputDatas[0].get());
        int cnt = 0;
        fastmath::fastMath.init();
        for (uint32_t i = 0; i < boxHeight; i++) {
            std::vector<std::vector<float>> tmpHeight;
            for (uint32_t j = 0; j < boxWidth; j++) {
                std::vector<float> tmpWidth;
                for (int k = 0; k < BOX_STRIDE; k++) {
                    tmpWidth.push_back(outputData[cnt++]);
                }
                tmpHeight.push_back(tmpWidth);
            }
            pixelsInfo.push_back(tmpHeight);
        }
        for (uint32_t i = 0; i < boxHeight; i++) {
            for (uint32_t j = 0; j < boxWidth; j++) {
                for (int k = 0; k < scoreStride; k++) {
                    pixelsInfo[i][j][k] = fastmath::sigmoid(pixelsInfo[i][j][k]);
                }
            }
        }
        for (uint32_t i = 0; i < boxHeight; i++) {
            for (uint32_t j = 0; j < boxWidth; j++) {
                if (pixelsInfo[i][j][0] > PIXEL_THRESHOLD) {
                    activationPixels.push_back(std::make_pair(i, j));
                }
            }
        }
    }

    void GenerateRegionList(std::vector<std::pair<int, int>> activationPixels,
                            std::vector<std::vector<std::pair<int, int>>> &regionList)
    {
        for (size_t i = 0; i < activationPixels.size(); i++) {
            bool merge = false;
            for (size_t k = 0; k < regionList.size(); k++) {
                bool mergeRes = ShouldMerge(regionList[k], activationPixels[i]);
                if (mergeRes) {
                    regionList[k].push_back(activationPixels[i]);
                    merge = true;
                }
            }
            if (merge == false) {
                regionList.push_back({activationPixels[i]});
            }
        }
    }

    void GetBoxCoordinates(std::pair<int, int> pixelIndex, float score,
                           const std::vector<std::vector<std::vector<float>>> &pixelsInfo,
                           std::vector<int> &quadList, std::vector<float> &totalScore)
    {
        const float headScore = 0.1;
        const float tailScore = 0.9;
        const int regionLen = 4;
        const int tailRegionOffset = 4;
        const int pixelSize = 4;
        const float axisOffset = 0.5;
        const int pixelAxisIndexOffset = 3;
        const int scoreIndex = 2;
        const float maxConfidenceThres = 0.9;

        if (score < maxConfidenceThres) {
            return;
        }

        float ithScore = pixelsInfo[pixelIndex.first][pixelIndex.second][scoreIndex];
        if (ithScore >= headScore && ithScore <= tailScore) {
            return;
        }

        int ith = 0;
        int startIndex = 0;
        int endIndex = 0;
        if (ithScore < headScore) {
            ith = 0;
            startIndex = 0;
            endIndex = regionLen;
        } else {
            ith = 1;
            startIndex = regionLen;
            endIndex = tailRegionOffset + regionLen;
        }
        for (int i = startIndex; i < endIndex; i++) {
            totalScore[i] += score;
        }
        float px = (pixelIndex.second + axisOffset) * pixelSize;
        float py = (pixelIndex.first + axisOffset) * pixelSize;
        float pv[regionLen];
        for (int i = 0; i < regionLen; i += TEXT_BOX_COORDINATES_DIM) {
            pv[i] = px + pixelsInfo[pixelIndex.first][pixelIndex.second][i + pixelAxisIndexOffset];
            pv[i + 1] = py + pixelsInfo[pixelIndex.first][pixelIndex.second][i + pixelAxisIndexOffset + 1];
        }

        for (int i = 0; i < regionLen; i++) {
            quadList[ith * regionLen + i] += score * pv[i];
        }
    }
}

void detect_postprocess(std::vector<std::shared_ptr<void>> outputDatas, uint32_t modelWidth, uint32_t modelHeight,
                       std::vector<std::vector<int>> &textBoxList)
{
    const float epsilon = 1e-4;
    const int pixelNum = 4;
    uint32_t boxWidth = 0;
    uint32_t boxHeight = 0;
    ComputeBoxSize(modelWidth, modelHeight, boxWidth, boxHeight);

    std::vector<std::vector<std::vector<float>>> pixelsInfo;
    std::vector<std::pair<int, int>> activationPixels;
    GeneratePixelsInfo(outputDatas, pixelsInfo, activationPixels, boxWidth, boxHeight);

    std::vector<std::vector<std::pair<int, int>>> regionList;
    GenerateRegionList(activationPixels, regionList);

    std::vector<std::vector<int>> groupVec;
    RegionGroup(regionList, groupVec);

    std::vector<std::vector<int>> quadList;
    std::vector<std::vector<float>> scoreList;
    for (size_t i = 0; i < groupVec.size(); i++) {
        quadList.push_back({0, 0, 0, 0, 0, 0, 0, 0});
        scoreList.push_back({0, 0, 0, 0});
        std::vector<float> totalScore = {0, 0, 0, 0, 0, 0, 0, 0};
        for (size_t j = 0; j < groupVec[i].size(); j++) {
            for (size_t k = 0; k < regionList[groupVec[i][j]].size(); k++) {
                float score = pixelsInfo[regionList[groupVec[i][j]][k].first][regionList[groupVec[i][j]][k].second][1];
                GetBoxCoordinates(regionList[groupVec[i][j]][k], score, pixelsInfo, quadList[i], totalScore);
            }
        }
        for (int w = 0; w < pixelNum; w++) {
            scoreList[i][w] = totalScore[w * TEXT_BOX_COORDINATES_DIM];
        }
        for (size_t z = 0; z < pixelNum * TEXT_BOX_COORDINATES_DIM; z++) {
            quadList[i][z] /= (totalScore[z] + epsilon);
        }
    }

    for (size_t i = 0; i < scoreList.size(); i++) {
        auto smallest = std::min_element(std::begin(scoreList[i]), std::end(scoreList[i]));
        if (*smallest > 0) {
            textBoxList.push_back(quadList[i]);
        }
    }
    return;
}