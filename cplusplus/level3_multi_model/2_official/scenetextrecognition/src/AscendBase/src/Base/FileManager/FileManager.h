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

#ifndef FILEMANAGER_H
#define FILEMANAGER_H
#include <dirent.h>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <algorithm>
#include <vector>
#include <memory>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <libgen.h>
#include <iostream>
#include <set>
#include "CommonDataType/CommonDataType.h"
#include "Log/Log.h"
#include "ErrorCode/ErrorCode.h"

#define BUF_SIZE 32U // Max buffer size
#define TWO (int)2
#define SLASH ( std::string)("/") // delimiter used to split path

mode_t SetFileDefaultUmask();
mode_t SetFileUmask(mode_t newUmask);
APP_ERROR ExistFile(const std::string &filePath);
APP_ERROR ExistDir(const std::string &dirPath);
std::vector<std::string> SplitPath(const std::string &str, const std::set<char>& delimiters);
APP_ERROR CreateDir(const std::string &dirPath);
void CreateDirRecursively(const std::string &file);
void CreateDirRecursivelyByFile(const std::string &file);
APP_ERROR ReadFile(const std::string &filePath, RawData &fileData);
APP_ERROR ReadFileWithOffset(const std::string &fileName, RawData &fileData, const uint32_t offset);
APP_ERROR ReadBinaryFile(const std::string &fileName, std::shared_ptr<uint8_t> &buffShared, int &buffLength);
std::string GetExtension(const std::string &filePath);
std::vector<std::string> ReadByExtension(const std::string &dirPath, const std::vector<std::string>& format);
std::string GetName(const std::string &filePath);
std::string GetParent(const std::string &filePath);
APP_ERROR ChangeDir(const std::string &dir);
void SaveFileAppend(const std::string &fileName, const std::string &stream, const int streamLength);
void SaveFileOverwrite(const std::string &fileName, const std::string &stream, const int streamLength);
void CopyFile(const std::string &srcFile, const std::string &destFile);
APP_ERROR SaveFileWithTimeStamp(std::shared_ptr<void> imageBuffer, uint32_t bufferSize, std::string folderName,
                                const std::string& fileName, const std::string& fileSuffix);
#endif
