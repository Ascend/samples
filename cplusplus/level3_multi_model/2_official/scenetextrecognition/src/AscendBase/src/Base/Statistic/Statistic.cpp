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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <FileManager/FileManager.h>

#include "Statistic.h"
#include "Log/Log.h"

bool Statistic::statisticEnable = false;
std::mutex Statistic::mutex_;
uint32_t Statistic::globalTimeCount = 0;
bool Statistic::globalTimeIsOver = false;
bool Statistic::globalTimeIsInit = false;
timeval Statistic::globalTimeStart = { 0, 0 };
timeval Statistic::globalTimeStop = { 0, 0 };
std::string Statistic::globalTimeModeName;
std::string Statistic::globalTimeFileToSave = DEFAUTL_SAVE_FILE;

const int SPLIT_LENGTH = 120;
const int INTERVAL_LENGTH_NAME = 40;
const int INTERVAL_LENGTH_DEFAULT = 15;
const double TIME_TRANS = 1000.0;
const int RUN_TIME_STATISTIC_PERIOD = 1000;
const int GLOBAL_TIME_STATISTIC_PERIOD = 2000;
const int DEFATL_RECORD_SIZE = 10000;

void Statistic::SetStatisticEnable(bool flag)
{
    statisticEnable = flag;
}

void Statistic::RunTimeStatisticStart(const std::string& modelName, uint32_t id, 
    bool autoShowResult, std::string fileToSave)
{
    if (Statistic::statisticEnable) {
        if (!runTimeIsInit_) {
            runTimeModelName_ = modelName;
            runTimeId_ = id;
            runTimeTotal_ = 0.0;
            runTimeCount_ = 0;
            runTimeIsOver_ = false;
            if (!fileToSave.empty()) {
                runTimeFileToSave_ = fileToSave;
            }
            if (autoShowResult) {
                std::thread t(Statistic::ShowRunTimeStatistic, this);
                t.detach();
            }
            runTimeIsInit_ = true;
            runTimeRecord_.clear();
            runTimeRecord_.reserve(DEFATL_RECORD_SIZE);
        }
        gettimeofday(&runTimeStart_, nullptr);
    }
}

void Statistic::RunTimeStatisticStop(uint32_t dynamicRunTimeCount)
{
    if (Statistic::statisticEnable) {
        gettimeofday(&runTimeStop_, nullptr);
        double tt = (runTimeStop_.tv_sec - runTimeStart_.tv_sec) * TIME_TRANS +
            (runTimeStop_.tv_usec - runTimeStart_.tv_usec) / TIME_TRANS;
        runTimeRecord_.push_back(std::make_pair(tt, std::max(dynamicRunTimeCount, 1U)));
        runTimeTotal_ += tt;
        while (runTimeIsOver_) {
            runTimeIsOver_ = false;
        }
        runTimeCount_ += std::max(dynamicRunTimeCount, 1U);
    }
}

void Statistic::ShowRunTimeStatistic(Statistic *statistic)
{
    if (statistic == nullptr) {
        LogError << "input statistic is null";
        return;
    }
    while (!statistic->runTimeIsOver_) {
        statistic->runTimeIsOver_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(RUN_TIME_STATISTIC_PERIOD));
    }
    if (statistic->runTimeCount_ == 0) {
        LogDebug << "the statistic is not start within a measurement period";
        return;
    }
    statistic->ShowStatisticResult();
    statistic->ShowStatisticRecord();
    while (statistic->runTimeIsInit_) {
            statistic->runTimeIsInit_ = false;
        }
    while (statistic->runTimeIsOver_) {
        statistic->runTimeIsOver_ = false;
    }
}

void Statistic::ShowStatisticResult()
{
    std::string split(SPLIT_LENGTH, '-');
    std::ostringstream ss;
    ss << std::endl << split << std::endl;
    ss.setf(std::ios::left);
    ss << std::setw(INTERVAL_LENGTH_NAME) << "model name" << std::setw(INTERVAL_LENGTH_DEFAULT) << "id"
       << std::setw(INTERVAL_LENGTH_DEFAULT) << "time(ms)" << std::setw(INTERVAL_LENGTH_DEFAULT) << "count"
       << std::setw(INTERVAL_LENGTH_DEFAULT) << "average(ms)"
       << "tps" << std::endl;
    ss << std::setw(INTERVAL_LENGTH_NAME) << runTimeModelName_ << std::setw(INTERVAL_LENGTH_DEFAULT) << runTimeId_
       << std::setw(INTERVAL_LENGTH_DEFAULT) << runTimeTotal_ << std::setw(INTERVAL_LENGTH_DEFAULT) << runTimeCount_
       << std::setw(INTERVAL_LENGTH_DEFAULT) << (runTimeTotal_ / runTimeCount_) << std::setw(INTERVAL_LENGTH_DEFAULT)
       << (runTimeCount_ / runTimeTotal_ * TIME_TRANS) << std::endl;
    ss.setf(std::ios::right);
    ss << split << std::endl << std::endl;

    std::lock_guard<std::mutex> locker(mutex_);
    std::cout << ss.str();
    CreateDirRecursively(runTimeFileToSave_.substr(0, runTimeFileToSave_.rfind('/')));
    SaveFileAppend(runTimeFileToSave_, ss.str().c_str(), ss.str().length());
}

void Statistic::ShowStatisticRecord()
{
    std::string split(SPLIT_LENGTH, '-');
    std::ostringstream ss;

    ss << std::endl << split << std::endl;
    ss.setf(std::ios::left);
    ss << std::setw(INTERVAL_LENGTH_NAME) << "model name" << std::setw(INTERVAL_LENGTH_DEFAULT) << "No."
       << std::setw(INTERVAL_LENGTH_DEFAULT) << "id" << std::setw(INTERVAL_LENGTH_DEFAULT) << "time(ms)"
       << std::setw(INTERVAL_LENGTH_DEFAULT) << "count" << std::endl;
    for (size_t i = 0; i < runTimeRecord_.size(); ++i) {
        ss << std::setw(INTERVAL_LENGTH_NAME) << runTimeModelName_ << std::setw(INTERVAL_LENGTH_DEFAULT)
           << ("No." + std::to_string(i)) << std::setw(INTERVAL_LENGTH_DEFAULT) << runTimeId_
           << std::setw(INTERVAL_LENGTH_DEFAULT) << runTimeRecord_[i].first << std::setw(INTERVAL_LENGTH_DEFAULT)
           << runTimeRecord_[i].second << std::endl;
    }
    ss.setf(std::ios::right);
    ss << split << std::endl << std::endl;
    SaveFileAppend(runTimeFileToSave_, ss.str().c_str(), ss.str().length());
}

void Statistic::GlobalTimeStatisticStart(const std::string& modelName, bool autoShowResult, 
    const std::string& fileToSave)
{
    if (Statistic::statisticEnable && !Statistic::globalTimeIsInit) {
        globalTimeIsOver = false;
        globalTimeCount = 0;
        Statistic::globalTimeModeName = modelName;
        Statistic::globalTimeFileToSave = fileToSave;
        if (autoShowResult) {
            std::thread t(Statistic::ShowGlobalTimeStatistic);
            t.detach();
        }
        Statistic::globalTimeIsInit = true;
        gettimeofday(&Statistic::globalTimeStart, nullptr);
    }
}

void Statistic::GlobalTimeStatisticStop()
{
    gettimeofday(&Statistic::globalTimeStop, nullptr);
    while (globalTimeIsOver) {
        globalTimeIsOver = false;
    }
    ++Statistic::globalTimeCount;
}

void Statistic::ShowGlobalTimeStatistic()
{
    while (!Statistic::globalTimeIsOver) {
        Statistic::globalTimeIsOver = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(GLOBAL_TIME_STATISTIC_PERIOD));
    }
    if (Statistic::globalTimeCount == 0) {
        LogDebug << "the statistic is not start within a measurement period";
        while (Statistic::globalTimeIsInit) {
            Statistic::globalTimeIsInit = false;
        }
        return;
    }
    double tt = (Statistic::globalTimeStop.tv_sec - Statistic::globalTimeStart.tv_sec) * TIME_TRANS +
        (Statistic::globalTimeStop.tv_usec - Statistic::globalTimeStart.tv_usec) / TIME_TRANS;

    std::string split(SPLIT_LENGTH, '-');
    std::ostringstream ss;

    ss << std::endl << std::endl << split << std::endl;
    ss.setf(std::ios::left);
    ss << std::setw(INTERVAL_LENGTH_NAME) << "model name" << std::setw(INTERVAL_LENGTH_DEFAULT) << "time(ms)"
       << std::setw(INTERVAL_LENGTH_DEFAULT) << "count" << std::setw(INTERVAL_LENGTH_DEFAULT) << "average(ms)"
       << "tps" << std::endl;
    ss << std::setw(INTERVAL_LENGTH_NAME) << Statistic::globalTimeModeName << std::setw(INTERVAL_LENGTH_DEFAULT)
       << tt << std::setw(INTERVAL_LENGTH_DEFAULT) << Statistic::globalTimeCount << std::setw(INTERVAL_LENGTH_DEFAULT)
       << (tt / Statistic::globalTimeCount) << std::setw(INTERVAL_LENGTH_DEFAULT)
       << (Statistic::globalTimeCount / tt * TIME_TRANS) << std::endl;
    ss.setf(std::ios::right);
    ss << split << std::endl << std::endl << std::endl;

    std::lock_guard<std::mutex> locker(mutex_);
    std::cout << ss.str();
    CreateDirRecursively(
        Statistic::globalTimeFileToSave.substr(0, Statistic::globalTimeFileToSave.rfind('/')));
    SaveFileAppend(Statistic::globalTimeFileToSave, ss.str().c_str(), ss.str().length());

    while (Statistic::globalTimeIsOver) {
        Statistic::globalTimeIsOver = false;
    }
    while (Statistic::globalTimeIsInit) {
        Statistic::globalTimeIsInit = false;
    }
}

double Statistic::GetRunTimeAvg(bool avg)
{
    double result = runTimeTotal_ / runTimeCount_;
    if (!avg) {
        result = runTimeTotal_;
    }
    return result;
}

double Statistic::GetGlobalTimeAvg(bool avg)
{
    double globalTimeTotal = (Statistic::globalTimeStop.tv_sec - Statistic::globalTimeStart.tv_sec) * TIME_TRANS +
        (Statistic::globalTimeStop.tv_usec - Statistic::globalTimeStart.tv_usec) / TIME_TRANS;
    double result = globalTimeTotal / globalTimeCount;
    if (!avg) {
        result = globalTimeTotal;
    }
    return result;
}