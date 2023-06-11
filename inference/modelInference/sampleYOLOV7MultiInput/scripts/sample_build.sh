#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

function main()
{
  echo "[INFO] Sample preparation"

  ret=`find ${ModelPath} -maxdepth 1 -name yolov7x.om 2> /dev/null`
  if [[ ${ret} ]];then
    echo "[INFO] The yolov7x.om already exists.start buiding"
  else
    echo "[ERROR] yolov7x.om does not exist, please follow the readme to convert the model and place it in the correct position!"
    return 1
  fi

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
  echo "[INFO] Sample preparation is complete"
}
main