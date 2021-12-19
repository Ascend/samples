#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

. ${THIRDPART_PATH}/common/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main ../data"
    # start runing
    running
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
