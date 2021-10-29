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

  wget -O ${ModelPath}/../data/dog_1024_688.jpg https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_picture_with_freetype/dog_1024_688.jpg --no-check-certificate
  
  find_model yolov3.om
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


