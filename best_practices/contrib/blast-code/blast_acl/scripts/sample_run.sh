#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] The sample starts to run"
  
  #running_command="./main 0 1 0 0 1"
  #data_command=""
  
  #running
  cd ../run/out/
  ./main
  if [ $? -ne 0 ];then
    return 1
  fi
}
main
