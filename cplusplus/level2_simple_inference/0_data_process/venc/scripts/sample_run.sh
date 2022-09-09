#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main ../data/dvpp_vpc_1920x1080_nv12.yuv"
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
