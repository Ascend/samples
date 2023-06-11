json_model="https://sharedata.obs.myhuaweicloud.com/blast-code/acl_op.json"

data_source_0="https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_0.bin"
data_source_1="https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_1.bin"
data_source_2="https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_2.bin"

opp_run="https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/custom_opp_Linux_aarch64.run"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../best_practices/contrib/blast-code/blast_acl/
project_path=${script_path}
common_script_dir=${script_path}/../../../../common
. ${common_script_dir}/testcase_common.sh

chmod_file="chmod 777 custom_opp_Linux_aarch64.run"
install_opp_run="./custom_opp_Linux_aarch64.run"
model_atc="atc --singleop=test_data/config/acl_op.json --soc_version=Ascend310 --output=op_models"
run_command="./main"

function setEnv() {
  export HOME=/home/HwHiAiUser
  export CPU_ARCH=`arch`
  export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
  export PATH=/usr/local/python3.7.5/bin:$PATH
  export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH 
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}
  export PYTHONPATH=${THIRDPART_PATH}/acllite:$PYTHONPATH
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest
  source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
}
function downloadDataOut()
{
  if [[ $(find ${project_path}/run/out/test_data/data/ -name ${data_source_0##*/})"x" = "x" ]];then
    wget -O ${project_path}/run/out/test_data/data/${data_source_0##*/}  ${data_source_0}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  if [[ $(find ${project_path}/run/out/test_data/data/ -name ${data_source_1##*/})"x" = "x" ]];then
    wget -O ${project_path}/run/out/test_data/data/${data_source_1##*/}  ${data_source_1}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  if [[ $(find ${project_path}/run/out/test_data/data/ -name ${data_source_2##*/})"x" = "x" ]];then
    wget -O ${project_path}/run/out/test_data/data/${data_source_2##*/}  ${data_source_2}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  return ${success}
}
function downloadModel()
{

  if [[ ${json_model}"x" != "x" ]];then
    wget -O ${project_path}/run/out/test_data/config/${json_model##*/}  ${json_model}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
    wget -O ${project_path}/run/out/${opp_run##*/}  ${opp_run}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  return ${success}
}

function main() {

    # 下载测试集
    downloadDataOut
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    # 下载模型
    downloadModel
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    setEnv

    # 安装run包
    cd ${project_path}/run/out/
    ${chmod_file}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi  
    ${install_opp_run}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi  

    # 转模型
    cd ${project_path}/run/out/
    ${model_atc}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi  

    buildproject
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi 

    cd ${project_path}/run/out/
    ${run_command}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main

