#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
conf_file_name="human_segmentation.conf"
common_script_dir=${THIRDPART_PATH}/common/
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] The sample starts to run"

  running_command="./main"
  data_command=" ../data/person.mp4"
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
