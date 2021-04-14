/*
异步正常拷贝数据 原长度和目标长度相同,展示4种情况:H2H\H2D\D2D\D2H  检查是否报错

编译一句话 l1 禁止使用 makefile
g++ -DENABLE_DVPP_INTERFACE  ./Async_Memcpy_Host2Device.cpp -o ./Async_Memcpy_Host2Device -I/usr/local/Ascend/ascend-toolkit/latest/acllib_linux.x86_64/include -I/usr/include/  -L/usr/local/Ascend/ascend-toolkit/latest/acllib_linux.x86_64/lib64/stub -lascendcl -lstdc++

*/

#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>
#include <stdint.h>
#include <map>
#include <iostream>
#include <signal.h>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

// 需要使用到静态公共库的函数名

// 全局资源
aclrtContext context = nullptr;						// 全局使用的ctx
aclError ret = 0;									// 定义主函数的返回状态
int32_t device_id = 0;								// 压力所在的deviceid
aclrtStream stream;									// 异步拷贝使用的stream

// 拷贝的输入输出buff
void* src_buffer = nullptr;						// device端输入内存地址
void* dst_buffer = nullptr;						// device端输出内存地址
size_t copy_len = 1024*1024*1024;				// 展示1M数据的拷贝

aclError init_fromwork()
{
	aclError ret = 0;
	// 初始化acl框架 set device和创建ctx
	ret = aclInit("./acl.json");					if (ret != ACL_ERROR_NONE){printf("acl init failed %d\n",ret);return -1;}					else{printf("acl init success \n");}
	ret = aclrtSetDevice(device_id);				if(ret!=ACL_ERROR_NONE){printf("aclrtSetDevice fail! %d \n",ret);return -1;}				else{printf("set device success \n");}
	ret = aclrtCreateContext(&context, device_id);	if(ret!=ACL_ERROR_NONE){printf("create ctx fail! %d \n",ret);return -1;}					else{printf("create context success \n");}
	ret = aclrtSetCurrentContext(context);			if(ret!=ACL_ERROR_NONE){printf("set thread to current context fail! \n");return -1;}		else{printf("set contex success \n");}
	ret = aclrtCreateStream(&stream);				if(ret!=ACL_ERROR_NONE){printf("create stream fail! \n");return -1;}						else{printf("create stream success \n");}
	
	// 申请主机和device的内存
	ret = aclrtMallocHost(&src_buffer, copy_len);	if(ret!=ACL_ERROR_NONE){printf("rt malloc host fail! \n");return -1;}						else{printf("malloc host success \n");}
	ret = aclrtMalloc(&dst_buffer, copy_len,ACL_MEM_MALLOC_HUGE_FIRST);	    
													if(ret!=ACL_ERROR_NONE){printf("rt malloc device fail! \n");return -1;}						else{printf("malloc device success \n");}
	return ACL_ERROR_NONE;
}

aclError uninit_formwork()
{
	//释放主机上的内存
	if(src_buffer != nullptr){aclrtFreeHost(src_buffer);}
	if(dst_buffer != nullptr){aclrtFree(dst_buffer);}
	ret = aclrtDestroyStream(stream);				if(ret!=ACL_ERROR_NONE){printf("destroy stream fail! \n");}									else{printf("destroy stream success \n");}
	ret = aclrtDestroyContext(context);				if(ret!=ACL_ERROR_NONE){printf("destroy context fail! \n");}								else{printf("destroy context success \n");}
	ret = aclrtResetDevice(device_id);				if(ret!=ACL_ERROR_NONE){printf("reset device fail! \n");}									else{printf("reset device success \n");}
	ret = aclFinalize();							if(ret!=ACL_ERROR_NONE){printf("aclFinalize fail! \n");}					    			else{printf("aclFinalize success \n");}
	return ACL_ERROR_NONE;
}

aclError copy_to_device()
{
	ret = aclrtMemcpyAsync(dst_buffer, copy_len, src_buffer, copy_len, ACL_MEMCPY_HOST_TO_DEVICE, stream);
	if(ret != ACL_ERROR_NONE){printf("Memcpy Async  fail! \n");return -1;} 																		else{printf("destroy stream success \n");};
	
	ret = aclrtSynchronizeStream(stream);			if(ret!=ACL_ERROR_NONE){printf("sync stream fail! %d \n",ret);return -1;}					else{printf("sync stream success \n");};
	return ACL_ERROR_NONE;
}

// l1 用例设计尽量简单,单个文件 解释清楚, 如果使用公共函数需要在头文件出写注释.
// 根本用例描述如何使用异步拷贝功能将数据拷贝到到device上
aclError main(int argc,char *argv[]){
	// 准备工作 封装到函数中 明确准备内容
	if(init_fromwork() != ACL_ERROR_NONE){printf("main init_fromwork failed !\n");return ACL_ERROR_UNINITIALIZE;}								else{printf("main init_fromwork success \n");}

	// 要展示的功能可以封装函数也可以直接放到main函数中直接写突出展示内容和返回效果.
	//展示拷贝功能
	if(copy_to_device() != ACL_ERROR_NONE){printf("copy_to_device failed !\n");return ACL_ERROR_UNINITIALIZE;}									else{printf("memcopy to device success \n");}

	// 收尾工作 封装到函数中 明确资源回收和其他动作
	if(uninit_formwork() !=ACL_ERROR_NONE){printf("uninit_formwork fail! \n");return -1;}														else{printf("uninit_formwork success\n");}
	return ACL_ERROR_NONE;
}
