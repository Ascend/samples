#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${THIRDPART_PATH}/common
conf_file_name="multi_channels_rtsp.conf"
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] The sample starts to run"

  running_command="./main"
  data_command=" "

  parse_presenter_view_ip

  if [ $? -ne 0 ];then
    return 1
  fi

  running_presenter
  if [ $? -ne 0 ];then
    return 1
  fi
}
main
