#/bin/bash
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

  if [ ! -f "${DataPath}/dvpp_output.yuv" ];then
    wget -O ${DataPath}/dvpp_output.yuv https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/jpege/dvpp_output.yuv --no-check-certificate
  fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi
    
  echo "[INFO] Sample preparation is complete"
}
main

