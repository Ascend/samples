/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STATISTIC_H
#define STATISTIC_H

#include <sys/time.h>
#include <cstdint>
#include <string>
#include <mutex>
#include <vector>
#include <memory>
#include <thread>

#define DEFAUTL_SAVE_FILE (std::string )("./logs/statistic.txt")
class Statistic {
public:
    Statistic()
        : runTimeCount_(0),
          runTimeId_(0),
          runTimeTotal_(0.0),
          runTimeIsOver_(false),
          runTimeIsInit_(false),
          runTimeStart_({ 0, 0 }),
          runTimeStop_({ 0, 0 }),
          runTimeModelName_(""),
          runTimeFileToSave_("") {};
    explicit Statistic(std::string modelName)
        : runTimeCount_(0),
          runTimeId_(0),
          runTimeTotal_(0.0),
          runTimeIsOver_(false),
          runTimeIsInit_(false),
          runTimeStart_({ 0, 0 }),
          runTimeStop_({ 0, 0 }),
          runTimeModelName_(""),
          runTimeFileToSave_("") {};
    ~Statistic() {};

    void RunTimeStatisticStart(const std::string& modelName, uint32_t id = 0, 
        bool autoShowResult = true, std::string fileToSave = DEFAUTL_SAVE_FILE);
    void RunTimeStatisticStop(uint32_t dynamicRunTimeCount = 0);
    double GetRunTimeAvg(bool avg = true);
    static void GlobalTimeStatisticStart(const std::string& modelName, 
        bool autoShowResult = true, const std::string& fileToSave = DEFAUTL_SAVE_FILE);
    static void GlobalTimeStatisticStop();
    static double GetGlobalTimeAvg(bool avg = true);
    void ShowStatisticResult();
    void ShowStatisticRecord();
    static void ShowRunTimeStatistic(Statistic *statistic);
    static void ShowGlobalTimeStatistic();
    static void SetStatisticEnable(bool flag);

    static bool statisticEnable;

private:
    uint32_t runTimeCount_;
    uint32_t runTimeId_;
    double runTimeTotal_;
    bool runTimeIsOver_;
    bool runTimeIsInit_;
    timeval runTimeStart_;
    timeval runTimeStop_;
    std::string runTimeModelName_;
    std::string runTimeFileToSave_;
    std::vector<std::pair<double, uint32_t>> runTimeRecord_ = {};

    static std::mutex mutex_;

    static uint32_t globalTimeCount;
    static bool globalTimeIsOver;
    static bool globalTimeIsInit;
    static timeval globalTimeStart;
    static timeval globalTimeStop;
    static std::string globalTimeModeName;
    static std::string globalTimeFileToSave;
};

#endif
