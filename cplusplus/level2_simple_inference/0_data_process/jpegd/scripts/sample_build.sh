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

  wget -O ${ModelPath}/../data/dog1_1024_683.jpg https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/jpegd/dog1_1024_683.jpg --no-check-certificate
  
  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

