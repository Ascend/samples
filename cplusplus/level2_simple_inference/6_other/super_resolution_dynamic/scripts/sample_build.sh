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

  if [ ! -f "${ModelPath}/../data/butterfly_GT.bmp" ];then
  wget -O ${ModelPath}/../data/butterfly_GT.bmp https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/super_resolution/test_image/butterfly_GT.bmp --no-check-certificate
  fi

  find_model FSRCNN_x3.om
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

