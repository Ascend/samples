#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"

function find_model()
{
  ret=`find ${ModelPath} -maxdepth 1 -name $1 2> /dev/null`
  if [[ ${ret} ]];then
    echo "[INFO] The $1 already exists.start buiding"
    return 0
  else
    echo "[ERROR] $1 does not exist, please follow the readme to convert the model and place it in the correct position!"
    return 1
  fi
}

function build()
{
  UserKernel=`arch`
  if [[ ${TargetKernel} = "x86" ]] || [[ ${TargetKernel} = "X86" ]];then
    TargetCompiler="g++"
    TargetKernel="x86"
  else
    if [[ ${UserKernel} == "x86_64" ]];then
      TargetCompiler="aarch64-linux-gnu-g++"
      TargetKernel="arm"
    else
      TargetCompiler="g++"
      TargetKernel="arm"
    fi
  fi
  if [ -d ${ScriptPath}/../build/intermediates/host ];then
    rm -rf ${ScriptPath}/../build/intermediates/host
  fi
    
  mkdir -p ${ScriptPath}/../build/intermediates/host
  cd ${ScriptPath}/../build/intermediates/host

  # Start compiling
  cmake ../../../src -DCMAKE_CXX_COMPILER=${TargetCompiler} -DCMAKE_SKIP_RPATH=TRUE -DUSE_LIBRARY=ON -DUSE_PRESENT=ON
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

function target_kernel()
{
  declare -i CHOICE_TIMES=0
  while [[ ${TargetKernel}"X" = "X" ]]
  do
    # three times choice 
    [[ ${CHOICE_TIMES} -ge 3 ]] && break || ((CHOICE_TIMES++))
    read -p "please input TargetKernel? [arm/x86]:" TargetKernel
    if [ ${TargetKernel}"z" = "armz" ] || [ ${TargetKernel}"z" = "Armz" ] || [ ${TargetKernel}"z" = "x86z" ] || [ ${TargetKernel}"z" = "X86z" ]; then
      echo "[INFO] input is normal, start preparation."
    else
      echo "[WARNING] The ${CHOICE_TIMES}th parameter input error!"
      TargetKernel=""
    fi
  done
  if [ ${TargetKernel}"z" = "z" ];then
    echo "[ERROR] TargetKernel entered incorrectly three times, please input arm/x86!"
    return 1
  else
    return 0
  fi
}

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi

    if [ ! -f "${ModelPath}/../data/car0.mp4" ];then
    wget -O ${ModelPath}/../data/car0.mp4 https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
  fi

  if [ ! -f "${ModelPath}/../data/car1.mp4" ];then
    wget -O ${ModelPath}/../data/car1.mp4 https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.mp4 --no-check-certificate
  fi

  if [ ! -f "${ModelPath}/../data/car1.jpg" ];then
    wget -O ${ModelPath}/../data/car1.jpg https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.jpg --no-check-certificate
  fi

  find_model yolov3.om
  if [ $? -ne 0 ];then
    return 1
  fi
  
  find_model color_dynamic_batch.om
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

