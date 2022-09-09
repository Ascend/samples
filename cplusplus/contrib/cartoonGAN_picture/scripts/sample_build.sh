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
  if [ ! -f "${ModelPath}/../data/test.jpg" ];then
    wget -O ${ModelPath}/../data/test.jpg https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/test_image/test.jpg --no-check-certificate
  fi
  find_model cartoonization.om
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

