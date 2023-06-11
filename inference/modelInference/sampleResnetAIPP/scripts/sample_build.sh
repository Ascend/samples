#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

function build()
{
  if [ -d ${ScriptPath}/../build/intermediates/host ];then
    rm -rf ${ScriptPath}/../build/intermediates/host
  fi

  mkdir -p ${ScriptPath}/../build/intermediates/host
  cd ${ScriptPath}/../build/intermediates/host

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
  cd - > /dev/null
}
function main()
{
  echo "[INFO] Sample preparation"

 ret=`find ${ModelPath} -maxdepth 1 -name resnet50.om 2> /dev/null`

   if [[ ${ret} ]];then
      echo "[INFO] The resnet50.om already exists.start buiding"
    else
      echo "[ERROR] resnet50.om does not exist, please follow the readme to convert the model and place it in the correct position!"
      return 1
    fi

  build
  if [ $? -ne 0 ];then
    return 1
  fi

  echo "[INFO] Sample preparation is complete"
}
main