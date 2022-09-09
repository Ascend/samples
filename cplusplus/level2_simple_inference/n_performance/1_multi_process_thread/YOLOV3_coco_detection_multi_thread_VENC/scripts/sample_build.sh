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

  if [ ! -f "${ModelPath}/../data/video1.mp4" ];then
    wget -O ${ModelPath}/../data/video1.mp4 https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_multi_thread_VENC/test_image/video1.mp4 --no-check-certificate
  fi

  if [ ! -f "${ModelPath}/../data/video2.mp4" ];then
    wget -O ${ModelPath}/../data/video2.mp4 https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_multi_thread_VENC/test_image/video2.mp4 --no-check-certificate
  fi

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

