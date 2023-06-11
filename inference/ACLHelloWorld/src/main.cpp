/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
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
# include <iostream>
#include "acl/acl.h"
# define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO] " fmt "\n", ##args)
# define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN] " fmt "\n", ##args)
# define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR] " fmt "\n", ##args)
using namespace std;

int main(int argc, char *argv[])
{
    uint32_t deviceId = 0;
    aclrtContext context = nullptr;
    aclrtStream stream = nullptr;
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Init Failed");
	}
	INFO_LOG("Acl Init Success");

    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Set Device Failed");
	}
    INFO_LOG( "Acl Set Device Success,Current DeviceID:%d", deviceId);

    ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Create Context Failed");
	}
	INFO_LOG("Acl Create Context Success");

    ret = aclrtCreateStream(&stream);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Create Stream Failed");
	}
	INFO_LOG("Acl Create Stream Success");
    /*
    * 业务执行
    */
    ret = aclrtDestroyStream(stream);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Destroy Stream Failed");
	}
	INFO_LOG("Acl Destroy Stream Success");

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Destroy Context Failed");
	}
	INFO_LOG("Acl Destroy Context success");

    ret = aclrtResetDevice(deviceId);
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Reset Device Failed");
	}
	INFO_LOG("Acl Reset Device Success");

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE){
		ERROR_LOG("Acl Finalize Failed");
	}
	INFO_LOG("Acl Finalize Success");
    return 0;
}
