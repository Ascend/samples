#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
conf_file_name="mask_detection.conf"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] The sample starts to run"

  running_command="python3.6 main.py"
  data_command=" ../data/mask.h264"
  parse_presenter_view_ip
  if [ $? -ne 0 ];then
    return 1
  fi

  running_presenter_python
  if [ $? -ne 0 ];then
    return 1
  fi
}
main
