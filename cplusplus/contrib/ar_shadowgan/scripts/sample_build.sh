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
  if [ ! -f "${ModelPath}/../data/test_A.png.bin" ];then
    wget -O ${ModelPath}/../data/test_A.png.bin https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/arshadow/data/test_A.png.bin --no-check-certificate
  fi
  if [ ! -f "${ModelPath}/../data/test_B.png.bin" ];then
    wget -O ${ModelPath}/../data/test_B.png.bin https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/arshadow/data/test_B.png.bin --no-check-certificate
  fi
  find_model model.om
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

