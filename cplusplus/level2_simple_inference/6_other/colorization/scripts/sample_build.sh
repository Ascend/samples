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
  
  if [ ! -f "${ModelPath}/../data/dog.png" ];then
    wget -O ${ModelPath}/../data/dog.png https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization/dog.png --no-check-certificate
  fi

  find_model colorization.om
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

