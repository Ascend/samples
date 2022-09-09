#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
common_script_dir=${ScriptPath}/../../../../common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

  mkdir -p ${ScriptPath}/../data

  if [ ! -f "${ModelPath}/../data/data.mp4" ];then
    wget -O ${ModelPath}/../data/data.mp4 https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/TextRecognize/data.mp4 --no-check-certificate
  fi

  if [ ! -f "${ModelPath}/../data/char_dict_en.json" ];then
    wget -O ${ModelPath}/../data/char_dict_en.json  https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/TextRecognize/char_dict_en.json --no-check-certificate
  fi

  if [ ! -f "${ModelPath}/../data/ord_map_en.json" ];then
    wget -O ${ModelPath}/../data/ord_map_en.json  https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/TextRecognize/ord_map_en.json --no-check-certificate
  fi

  find_model dbnet.om
  if [ $? -ne 0 ];then
    return 1
  fi

  find_model crnn_static.om
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

