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

#ifndef INC_COMMAND_LINE_H
#define INC_COMMAND_LINE_H

#include <unistd.h>
#include "CommandParser/CommandParser.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "Statistic/Statistic.h"

// parameters of command line
struct CmdParams {
    int logLevel;
    bool statEnable;
};

APP_ERROR ParseACommandLine(int argc, const char *argv[], CmdParams &cmdParams)
{
    LogDebug << "Begin to parse and check command line.";
    CommandParser option;

    option.add_option("-sample_command", "", "./ocr");
    option.add_option("--log_level", "1", "debug level:0-debug, 1-info, 2-warn, 3-error, 4-fatal, 5-off.");
    option.add_option("--stats", "false", "to enable the statistic, false-disabled statistics, true-enable statistics");

    option.ParseArgs(argc, argv);
    cmdParams.logLevel = option.GetIntOption("--log_level");

    // check invalidity of input parameters
    if (cmdParams.logLevel < AtlasAscendLog::LOG_LEVEL_DEBUG || cmdParams.logLevel > AtlasAscendLog::LOG_LEVEL_NONE) {
        LogError << "Please check invalid parameter --log_level, is not in [" << AtlasAscendLog::LOG_LEVEL_DEBUG \
                 << ", " << AtlasAscendLog::LOG_LEVEL_NONE << "].";
        return APP_ERR_COMM_OUT_OF_RANGE;
    }
    cmdParams.statEnable = option.GetBoolOption("--stats");
    Statistic::SetStatisticEnable(cmdParams.statEnable);

    return APP_ERR_OK;
}

void SetLogLevel(int debugLevel)
{
    switch (debugLevel) {
        case AtlasAscendLog::LOG_LEVEL_DEBUG:
            AtlasAscendLog::Log::LogDebugOn();
            break;
        case AtlasAscendLog::LOG_LEVEL_INFO:
            AtlasAscendLog::Log::LogInfoOn();
            break;
        case AtlasAscendLog::LOG_LEVEL_WARN:
            AtlasAscendLog::Log::LogWarnOn();
            break;
        case AtlasAscendLog::LOG_LEVEL_ERROR:
            AtlasAscendLog::Log::LogErrorOn();
            break;
        case AtlasAscendLog::LOG_LEVEL_FATAL:
            AtlasAscendLog::Log::LogFatalOn();
            break;
        case AtlasAscendLog::LOG_LEVEL_NONE:
            AtlasAscendLog::Log::LogAllOff();
            break;
        default:
            break;
    }
}

#endif
