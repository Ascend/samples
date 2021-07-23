#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
. ${script_path}/../../../../../common/sample_common.sh

function main()
{
    echo "[INFO] The sample starts to run"

    running_command="./main ../data"
    # start runing
    running_picture
    if [ $? -ne 0 ];then
        return 1
    fi
}
main
