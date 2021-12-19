#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main ../data/wood_rabbit_1024_1068_nv12.yuv 1024 1068 ./output/output.yuv 224 224"
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
