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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <FileManager/FileManager.h>

#include "Log.h"

namespace AtlasAscendLog {
const int TIME_SIZE = 32;
const int TIME_DIFF = 28800; // 8 hour
const int BYTES6 = 6;
uint32_t Log::logLevel = LOG_LEVEL_INFO;
std::vector<std::string> Log::levelString({ "[Debug]", "[Info ]", "[Warn ]", "[Error]", "[Fatal]" });
std::mutex Log::mutex;
std::string Log::logFile = "./logs/log.log"; // default log file, for linux

Log::Log(const std::string& file, const std::string& function, int line, uint32_t level)
    : myLevel_(level), file_(file), function_(function), line_(line)
{
}

Log::~Log()
{
    if (myLevel_ >= logLevel) {
        std::lock_guard<std::mutex> locker(mutex);
        // cout to screen
        std::cout << ss_.str() << std::endl;
        // log to the file
        CreateDirRecursivelyByFile(logFile);
        size_t posDot = logFile.rfind(".");
        std::string dataString = date_.substr(date_.find('[') + 1, date_.find(' ') - date_.find('[') - 1);
        std::string file = logFile.substr(0, posDot) + "_" + dataString + logFile.substr(posDot);
        std::ofstream fs(file, std::ios::app);
        if (!fs) {
            std::cout << "open file " << file << " fail" << std::endl;
            return;
        }
        fs << ss_.str() << std::endl;
        fs.close();
    }
};

std::ostringstream &Log::Stream()
{
    if (myLevel_ >= logLevel) {
        struct timeval time = { 0, 0 };
        gettimeofday(&time, nullptr);
        time_t timep = time.tv_sec + TIME_DIFF;
        struct tm *ptm = gmtime(&timep);
        char timeString[TIME_SIZE] = {0};
        strftime(timeString, TIME_SIZE, "[%F %X:", ptm);
        date_ = timeString;

        ss_.fill('0');
        ss_ << levelString[myLevel_] << timeString << std::setw(BYTES6) << time.tv_usec << "]";

        std::string fileName = file_.substr(file_.rfind('/') + 1); // for linux
        ss_ << "[" << fileName << " " << function_ << ":" << line_ << "] ";
    }
    return ss_;
}

void Log::LogDebugOn()
{
    logLevel = LOG_LEVEL_DEBUG;
    return;
}
void Log::LogInfoOn()
{
    logLevel = LOG_LEVEL_INFO;
    return;
}
void Log::LogWarnOn()
{
    logLevel = LOG_LEVEL_WARN;
    return;
}
void Log::LogErrorOn()
{
    logLevel = LOG_LEVEL_ERROR;
    return;
}
void Log::LogFatalOn()
{
    logLevel = LOG_LEVEL_FATAL;
    return;
}
void Log::LogAllOn()
{
    logLevel = LOG_LEVEL_DEBUG;
    return;
}
void Log::LogAllOff()
{
    logLevel = LOG_LEVEL_NONE;
    return;
}

#define LOG_DEBUG Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_DEBUG)
#define LOG_INFO Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_INFO)
#define LOG_WARN Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_WARN)
#define LOG_ERROR Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_ERROR)
#define LOG_FATAL Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_FATAL)
} // namespace AtlasAscendLog