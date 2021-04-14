# Copyright (c) Huawei Technologies Co., Ltd. 2020. All rights reserved.
cmake_minimum_required(VERSION 3.5.0)

# Header path
include_directories(../../../models/jsoncpp/dist)
set(JSON ../../../models/jsoncpp/dist)

set(DATA_PROCESS_FILES
    data_process.h
    data_process.cpp
    ${JSON}/json/json.h
    ${JSON}/jsoncpp.cpp
    )

add_library(dataprocess ${DATA_PROCESS_FILES})
