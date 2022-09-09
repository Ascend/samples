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

  if [ ! -f "${DataPath}/dog1_1024_683.jpg" ];then
    wget -O ${DataPath}/dog1_1024_683.jpg https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/jpegd/dog1_1024_683.jpg --no-check-certificate
  fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

