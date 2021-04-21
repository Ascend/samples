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

function running()
{
    Kernel=`uname -m`
    if [[ ${Kernel} = "x86_64" ]];then
        Targetkernel="x86"
    else
        Targetkernel="arm"
    fi
    export LD_LIBRARY_PATH=${HOME}/ascend_ddk/${Targetkernel}/lib:${HOME}/Ascend/acllib/lib64:$LD_LIBRARY_PATH
    cd ${ScriptPath}/../out
    rm -rf output
    mkdir output
    ./main ../data
    if [ $? -ne 0 ];then
        echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
        return 1
    else
        echo "[INFO] The program runs successfully, please view the result file in the ${ScriptPath}../out/output directory!"
        return 0
    fi
}

# ********************** run sample ************************************
function main()
{
    echo "[INFO] The sample starts to run"

    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
