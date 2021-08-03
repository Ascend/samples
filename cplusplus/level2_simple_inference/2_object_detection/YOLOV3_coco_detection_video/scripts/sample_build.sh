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

  wget -O ${ModelPath}/../data/detection.mp4 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_video/detection.mp4 --no-check-certificate
  
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

