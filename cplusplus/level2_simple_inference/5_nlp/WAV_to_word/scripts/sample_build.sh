#/bin/bash
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
  
  if [ ! -f "${ModelPath}/../data/nihao.wav" ];then
    wget -O ${ModelPath}/../data/nihao.wav https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/WAV_to_word/nihao.wav --no-check-certificate
  fi
  if [ ! -f "${ModelPath}/../data/xinpian.wav" ];then
    wget -O ${ModelPath}/../data/xinpian.wav https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/WAV_to_word/xinpian.wav --no-check-certificate
  fi

  find_model voice.om
  if [ $? -ne 0 ];then
    return 1
  fi
    
  build
  if [ $? -ne 0 ];then
    return 1
  fi

  python3 ${ScriptPath}/preparedata.py
    
  echo "[INFO] Sample preparation is complete"
}
main

