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

  if [ ! -f "${DataPath}/wood_rabbit_1024_1068_nv12.yuv" ];then
    wget -O ${DataPath}/wood_rabbit_1024_1068_nv12.yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/resize/test_image/wood_rabbit_1024_1068_nv12.yuv --no-check-certificate
  fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

