#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
DataPath="${ScriptPath}/../data"
common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

  if [ ! -f "${DataPath}/detection.mp4" ];then
    wget -O ${DataPath}/detection.mp4 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/venc/detection.mp4 --no-check-certificate
  fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

