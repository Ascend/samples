/**
* @file op_execute.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef OP_EXECUTE_H
#define OP_EXECUTE_H
#include "op_test_desc.h"
bool OpExecute(OpTestDesc &opDesc, uint32_t deviceId = 0);
#endif