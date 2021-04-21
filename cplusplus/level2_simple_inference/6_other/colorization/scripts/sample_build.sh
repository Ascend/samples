#!/bin/bash
#
#   =======================================================================
#
# Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   1 Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   2 Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   3 Neither the names of the copyright holders nor the names of the
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#   =======================================================================
# ************************Variable*********************************************  
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
TargetKernel=$1

function ModelConvert()
{
    wget -O ${ModelPath}/../data/dog.png https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization/dog.png --no-check-certificate
    # Determine whether there is a better om model
    ret=`find ${ModelPath} -maxdepth 1 -name "colorization.om" 2> /dev/null`
    if [[ ${ret} ]];then
        echo "[INFO] The colorization.om already exists.start buiding"
        return 0
    else
        echo "[ERROR] colorization.om does not exist, please follow the readme to convert the model and place it in the correct position!"
	return 1
    fi
}

function BuildSample()
{

    if [[ ${TargetKernel} = "x86" ]] || [[ ${TargetKernel} = "X86" ]];then
        TargetCompiler="g++"
        TargetKernel="x86"
        export DDK_PATH=${HOME}/Ascend/ascend-toolkit/latest
    else
        TargetCompiler="aarch64-linux-gnu-g++"
        TargetKernel="arm"
        export DDK_PATH=${HOME}/Ascend/ascend-toolkit/latest/arm64-linux
    fi

    if [ -d ${ScriptPath}/../build/intermediates/host ];then
        rm -rf ${ScriptPath}/../build/intermediates/host
    fi
    
    mkdir -p ${ScriptPath}/../build/intermediates/host
    cd ${ScriptPath}/../build/intermediates/host

    # Start compiling
    export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    cmake ../../../src -DCMAKE_CXX_COMPILER=${TargetCompiler} -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "[ERROR] cmake error, Please check your environment!"
        return 1
    fi
    make
    if [ $? -ne 0 ];then
        echo "[ERROR] build failed, Please check your environment!"
        return 1
    fi
    cd -
}

function main()
{
    echo "[INFO] Sample preparation"
    
    if [[ ${TargetKernel} = "arm" ]] || [[ ${TargetKernel} = "Arm" ]] || [[ ${TargetKernel} = "x86" ]] || [[ ${TargetKernel} = "X86" ]] ;then
        echo "[INFO] Input is normal, start preparation"
    else    
        echo "[ERROR] The input parameter is wrong, please input TargetKernel(arm/x86)"
        return 1;
    fi
    
    cd ${ScriptPath}
    # Model Convert
    ModelConvert
    if [ $? -ne 0 ];then
        return 1
    fi
    
    # Build sample
    BuildSample
    if [ $? -ne 0 ];then
        return 1
    fi
    
    echo "[INFO] Sample preparation is complete"
}
main

