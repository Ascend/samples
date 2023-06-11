onnx_model_1="https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_1.onnx"
onnx_model_2="https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_2.onnx"
onnx_model_3="https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_3.onnx"
model_name_1="tf_model_1"
model_name_2="tf_model_2"
model_name_3="tf_model_3"

data_out_1="https://sharedata.obs.myhuaweicloud.com/depression_detection/audio_process/audio_feature01.txt"
data_out_2="https://sharedata.obs.myhuaweicloud.com/depression_detection/audio_process/audio_feature02.txt"

script_path_temp="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
script_path=${script_path_temp}/../../best_practices/contrib/depression_detection/inference
project_path=${script_path}
common_script_dir=${script_path}/../../../../common

model_1_atc="atc --model=${onnx_model_1##*/} --framework=5 --input_format=NCHW --output=${model_name_1} --input_shape="conv2d_4_input:1,1,20,1" --soc_version=Ascend310"
model_2_atc="atc --model=${onnx_model_2##*/} --framework=5 --input_format=NCHW --output=${model_name_2} --input_shape="conv2d_8_input:1,1,20,1" --soc_version=Ascend310"
model_3_atc="atc --model=${onnx_model_3##*/} --framework=5 --input_format=NCHW --output=${model_name_3} --input_shape="conv2d_88_input:1,1,20,1" --soc_version=Ascend310"

run_command="python3.6 postprocess.py --feature_dir ./dataout/ --model_path ./model/"

. ${common_script_dir}/testcase_common.sh

function setEnv() {

	source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
	export CPU_ARCH=`arch`
	export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
	export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  #运行时链接库文件
	export PATH=/usr/local/python3.7.5/bin:$PATH
	export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest #CANN软件安装后文件存储路径
}

function downloadDataOut()
{
  if [[ $(find ${project_path}/dataout -name ${data_out_1##*/})"x" = "x" ]];then
    wget -O ${project_path}/dataout/${data_out_1##*/}  ${data_out_1}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  if [[ $(find ${project_path}/dataout -name ${data_out_2##*/})"x" = "x" ]];then
    wget -O ${project_path}/dataout/${data_out_2##*/}  ${data_out_2}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  return ${success}
}

function downloadModel()
{
  if [[ ${onnx_model_1}"x" != "x" ]] && [[ ${onnx_model_2}"x" != "x" ]] && [[ ${onnx_model_3}"x" != "x" ]];then
    wget -O ${project_path}/model/${onnx_model_1##*/}  ${onnx_model_1}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
    wget -O ${project_path}/model/${onnx_model_2##*/}  ${onnx_model_2}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
    wget -O ${project_path}/model/${onnx_model_3##*/}  ${onnx_model_3}  --no-check-certificate
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

    downloadModel
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    setEnv
    # 转模型
    cd ${project_path}/model/
    ${model_1_atc}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    ${model_2_atc}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    ${model_3_atc}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi
    
    # 运行
    cd ${project_path}/
    ${run_command}
    if [ $? -ne 0 ];then
        return ${inferenceError}
    fi

    return ${success}
}
main
