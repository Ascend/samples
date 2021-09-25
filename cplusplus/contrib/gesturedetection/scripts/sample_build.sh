#!/bin/bash
ScriptPath="$( cd "$(dirname "$BASH_SOURCE")" ; pwd -P )"
ModelPath="${ScriptPath}/../model"
. ${ScriptPath}/../../../../common/sample_common.sh

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/gesturedetection/test_image/"
project_name="cplusplus_Gesturedetection"

project_path=${ScriptPath}/..

function main()
{
  echo "[INFO] Sample preparation"

  target_kernel
  if [ $? -ne 0 ];then
    return 1
  fi


    for ((i=0; i<50; i ++))
    do
    wget -O ${project_path}/data/$i".jpg"  ${data_source}$i".jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
            echo "Download jpg failed, please check network."
            return 1
        fi
    done

 # wget -O ${ModelPath}/../data/0.jpg wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/gesturedetection/test_image/0.jpg --no-check-certificate
  
  find_model pose_deploy.om
  find_model stgcn_fps30_sta_ho_ki4.om

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

