#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"

function main()
{
    echo "[INFO] The sample starts to run"
    running_command="./main"
    cd ${ScriptPath}/../out
    ${running_command}
    if [ $? -ne 0 ];then
        return 1
    else
        echo "[INFO] The program runs successfully"
        return 0
    fi
}
main
