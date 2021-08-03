#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

. ${ScriptPath}/../../../../../common/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main"
    # start runing
    running_picture
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
