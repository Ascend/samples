#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
. ${ScriptPath}/../../../../../common/sample_common.sh

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

  wget -O ${ModelPath}/../data/wood_rabbit_1024_1068_nv12.yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/crop/test_image/wood_rabbit_1024_1068_nv12.yuv --no-check-certificate
  
  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

