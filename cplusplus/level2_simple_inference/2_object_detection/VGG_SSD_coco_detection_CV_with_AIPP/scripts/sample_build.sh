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
  
  if [ ! -f "${ModelPath}/../data/bicycle.jpg" ];then
    wget -O ${ModelPath}/../data/bicycle.jpg https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/VGG_SSD_coco_detection_CV_with_AIPP/bicycle.jpg --no-check-certificate
  fi
  
  find_model vgg_ssd.om
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

