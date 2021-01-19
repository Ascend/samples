# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
cmake_minimum_required(VERSION 3.5.0)

# Header path
include_directories(../../tf_total_sentiment/jsoncpp/dist)

set(DATA_PROCESS_FILES
	data_process.h
	data_process.cpp
	../tf_total_sentiment/jsoncpp/dist/json/json.h
	../tf_total_sentiment/jsoncpp/dist/jsoncpp.cpp
	)

add_library(dataprocess 
			${DATA_PROCESS_FILES}
			)