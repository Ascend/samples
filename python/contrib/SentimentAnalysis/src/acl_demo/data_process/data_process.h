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

* File data_process.h
* Description: handle data process
*/
#ifndef DATA_PROCESS_H_
#define DATA_PROCESS_H_

#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <iterator>

#include "json/json.h"

using namespace std;

template<class T1, class T2>
class Vocab {
public:
	Vocab(const string& wordVocabFileName);
	~Vocab();
	Json::Value ReadVocb(const string& wordVocabFileName);
	T2 w2i(const T1& key) const;

private:
	map<T1, T2> maps_;
};

int ParseJsonFromString(string& str);

vector<string> split_chinese(string s);

template<class T1, class T2>
Vocab<T1, T2>::Vocab(const string& wordVocabFileName) {
	const Json::Value& value = ReadVocb(wordVocabFileName);
	Json::Value::Members members = value.getMemberNames();
	for (Json::Value::Members::iterator it = members.begin(); it != members.end(); it++)
	{
		Json::ValueType vt = value[*it].type();
		switch (vt)
		{
		case Json::intValue:
		{
			int intTmp = value[*it].asInt();
			maps_.insert(pair<string, int>(*it, intTmp));
			break;
		}
		default:
		{
			break;
		}
		}
	}
}

template<class T1, class T2>
Vocab<T1, T2>::~Vocab() {}

template<class T1, class T2>
Json::Value Vocab<T1, T2>::ReadVocb(const string& wordVocabFileName) {
	Json::Value value;
	std::ifstream ifs;
	ifs.open(wordVocabFileName);
	Json::CharReaderBuilder builder;
	builder["collectComments"] = false;
	JSONCPP_STRING errs;
	if (!parseFromStream(builder, ifs, &value, &errs)) {
		std::cout << errs << std::endl;
	}
	ifs.close();

	return value;
}

template<class T1, class T2>
T2 Vocab<T1, T2>::w2i(const T1& key) const {
	auto it = maps_.find(key);
	if (it == maps_.end())
		return 1;
	return it->second;
}

#endif //DATA_PROCESS_H_
