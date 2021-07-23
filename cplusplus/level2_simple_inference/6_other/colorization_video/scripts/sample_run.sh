#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
conf_file_name="colorization.conf"
. ../../../../../common/sample_common.sh

function main()
{
  echo "[INFO] The sample starts to run"

  running_command="./main"
  data_command=" ../data/black-white_video.mp4"
  parse_presenter_view_ip
  if [ $? -ne 0 ];then
    return 1
  fi

  running_video
  if [ $? -ne 0 ];then
    return 1
  fi
}
main
