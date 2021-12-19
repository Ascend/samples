#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

  if [ ! -f "${ModelPath}/../data/person.mp4" ];then
  wget -O ${ModelPath}/../data/person.mp4 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_rtsp/person.mp4 --no-check-certificate
  fi
  
  find_model face_detection.om
  if [ $? -ne 0 ];then
    return 1
  fi
    
  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

