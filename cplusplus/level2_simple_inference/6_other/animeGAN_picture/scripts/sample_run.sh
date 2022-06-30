#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh
shape=$1

function main()
{
  echo "[INFO] The sample starts to run"
  if [[ $shape != 256 ]] && [[ $shape != 512 ]] && [[ $shape != 1024 ]];then
    echo "[ERROR] The input parameter is incorrect, please enter 256/512/1024!"
    return 1
  fi
  running_command="./main ../data $shape"
  data_command=" "

  running
  if [ $? -ne 0 ];then
    return 1
  fi
}
main
