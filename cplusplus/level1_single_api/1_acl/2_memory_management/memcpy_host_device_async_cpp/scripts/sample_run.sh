#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"

common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main --release_cycle 2 --number_of_cycles 2\
    --device_id 0 --memory_size 10485760"
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
