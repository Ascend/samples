#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main "
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
