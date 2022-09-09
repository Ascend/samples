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

  wget -O ${ModelPath}/../data/dog1_1024_683.bin https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch/dog1_1024_683.bin --no-check-certificate
  wget -O ${ModelPath}/../data/dog2_1024_683.bin https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/googlenet_imagenet_dynamic_batch/test_image/test2.bin --no-check-certificate
  
  find_model googlenet_dynamicbatch.om
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

