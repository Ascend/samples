#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/model"

common_script_dir=${THIRDPART_PATH}/common
. ${common_script_dir}/sample_common.sh

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

  wget -O ${ModelPath}/../data/dog1_1024_683.jpg https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg --no-check-certificate

  cd ${ScriptPath}/data
  python3 ../script/transferPic.py

  find_model resnet50.om
  if [ $? -ne 0 ];then
    return 1
  fi


  if [ -d ${ScriptPath}/build/intermediates/host ];then
    rm -rf ${ScriptPath}/build/intermediates/host
  fi
 
  mkdir -p ${ScriptPath}/build/intermediates/host
  cd ${ScriptPath}/build/intermediates/host

  # Start compiling
  cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
  if [ $? -ne 0 ];then
    echo "[ERROR] cmake error, Please check your environment!"
    return 1
  fi
  make
  if [ $? -ne 0 ];then
    echo "[ERROR] build failed, Please check your environment!"
    return 1
  fi

#  if [ $? -ne 0 ];then
#    return 1
#  fi

  echo "[INFO] Sample preparation is complete"
}
main
