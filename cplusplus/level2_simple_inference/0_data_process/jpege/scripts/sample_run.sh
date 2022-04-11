#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
common_script_dir=${THIRDPART_PATH}/common

function running()
{
    Kernel=`uname -m`
   if [[ ${Kernel} = "x86_64" ]];then
        Targetkernel="x86"
    else
        Targetkernel="arm"
    fi
    cd ${ScriptPath}/../out
    rm -rf output
    mkdir output
    ${running_command}
    if [ $? -ne 0 ];then
        echo "[ERROR] The program failed to run, please check the log in the /var/log/npu/slog/host-0 directory!"
        return 1
    else
        echo "[INFO] The program runs successfully, please view the result file in the ${ScriptPath}/../out/output directory!"
        return 0
    fi
}

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main ../data/dvpp_output.yuv 1024 688"
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
