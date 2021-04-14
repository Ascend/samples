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

#ifndef LOG_H
#define LOG_H

#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace AtlasAscendLog {
// log level
enum LogLevels {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO = 1,
    LOG_LEVEL_WARN = 2,
    LOG_LEVEL_ERROR = 3,
    LOG_LEVEL_FATAL = 4,
    LOG_LEVEL_NONE
};

class Log {
public:
    Log(const std::string& file, const std::string& function, int line, uint32_t level);
    ~Log();
    std::ostringstream &Stream();
    // log switch, turn on and off both screen and file log of special level.
    static void LogDebugOn();
    static void LogInfoOn();
    static void LogWarnOn();
    static void LogErrorOn();
    static void LogFatalOn();
    static void LogAllOn();
    static void LogAllOff();

private:
    std::ostringstream ss_;
    uint32_t myLevel_;
    std::string date_;
    std::string file_;
    std::string function_;
    int line_;

    static uint32_t logLevel;
    static std::vector<std::string> levelString;
    static std::mutex mutex;
    static std::string logFile;
};
} // namespace AtlasAscendLog

#define LogDebug AtlasAscendLog::Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_DEBUG).Stream()
#define LogInfo AtlasAscendLog::Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_INFO).Stream()
#define LogWarn AtlasAscendLog::Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_WARN).Stream()
#define LogError AtlasAscendLog::Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_ERROR).Stream()
#define LogFatal AtlasAscendLog::Log(__FILE__, __FUNCTION__, __LINE__, AtlasAscendLog::LOG_LEVEL_FATAL).Stream()
#define LOG(security) AtlasAscendLog::LOG_##security.Stream()

#endif