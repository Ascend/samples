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

  wget -O ${ModelPath}/../data/cat.mp4 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/googlenet_imagenet_video/cat.mp4 --no-check-certificate
  
  find_model googlenet.om
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

