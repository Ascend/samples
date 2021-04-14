/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File data_process.cpp
* Description: handle data process
*/
#include "data_process.h"

int ParseJsonFromString(string& str)
{
    const auto rawStrLength = static_cast<int>(str.length());
    Json::Value root;
    JSONCPP_STRING err;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    
    if (!reader->parse(str.c_str(), str.c_str() + rawStrLength, &root, &err)) 
    {
      std::cout << "string parsing error!!!" << std::endl;
      return EXIT_FAILURE;
    }

    return 0;
}

vector<string> split_chinese(string s) {
    vector<string> words;
    for (size_t i = 0; i < s.length();) {
        int cplen = 1;
        // the following if-statements are referred to https://en.wikipedia.org/wiki/UTF-8#Description
        if ((s[i] & 0xf8) == 0xf0)      // 11111000, 11110000
            cplen = 4;
        else if ((s[i] & 0xf0) == 0xe0) // 11100000
            cplen = 3;
        else if ((s[i] & 0xe0) == 0xc0) // 11000000
            cplen = 2;
        if ((i + cplen) > s.length())
            cplen = 1;
        words.push_back(s.substr(i, cplen));
        i += cplen;
    }
    return words;
}