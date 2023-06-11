#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
Src=${ScriptPath}/../src


function main()
{
    cd ${Src}
    echo "[INFO] The sample starts to run"
    running_command="python3.7 sampleResnetQuickStart.py"
    ${running_command}
    if [ $? -ne 0 ];then
        return 1
    fi
}
main