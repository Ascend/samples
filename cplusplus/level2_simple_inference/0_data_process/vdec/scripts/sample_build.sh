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

  if [ ! -f "${DataPath}/vdec_h265_1frame_rabbit_1280x720.h265" ];then
    wget -O ${DataPath}/vdec_h265_1frame_rabbit_1280x720.h265 https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdec/vdec_h265_1frame_rabbit_1280x720.h265 --no-check-certificate
  fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

