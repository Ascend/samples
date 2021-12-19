#!/bin/bash
declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function downloadDataWithVerifySource()
{

  mkdir -p ${project_path}/data/
  if [[ $(find ${project_path}/data -name ${data_name})"x" = "x" ]];then
    wget -O ${project_path}/data/${data_name}  ${data_source}${data_name}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi

  if [[ ${verify_source}"x" != "x" ]];then
    mkdir -p ${project_path}/verify_image/
    if [[ $(find ${project_path}/verify_image -name ${verify_name})"x" = "x" ]];then
      wget -O ${project_path}/verify_image/${verify_name} ${verify_source}${verify_name} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "download verify data failed, please check Network."
        return ${inferenceError}
      fi
    fi
  fi
  
  return ${success}

}
function downloadData2()
{
  if [[ $(find ${project_path}/data -name ${data_name2})"x" = "x" ]];then
    wget -O ${project_path}/data/${data_name2}  ${data_source}${data_name2}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  
  return ${success}
}
function downloadVerifySource2()
{
  if [[ $(find ${project_path}/verify_image -name ${verify_name2})"x" = "x" ]];then
    wget -O ${project_path}/verify_image/${verify_name2}  ${verify_source}${verify_name2}  --no-check-certificate
    if [ $? -ne 0 ];then
      echo "download test data failed, please check Network."
      return ${inferenceError}
    fi
  fi
  
  return ${success}
}
function setEnv() {
  source /home/HwHiAiUser/.bashrc
  source /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
}
function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    if [[ ${caffe_prototxt}"x" != "x" ]] && [[ ${caffe_model}"x" != "x" ]];then
      wget -O ${project_path}/model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install caffe_prototxt failed, please check Network."
        return ${inferenceError}
      fi

      wget -O ${project_path}/model/${caffe_model##*/} ${caffe_model} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install caffe_model failed, please check Network."
        return ${inferenceError}
      fi
    elif [[ ${tf_model}"x" != "x" ]];then
      wget -O ${project_path}/model/${tf_model##*/} ${tf_model} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install tf_model failed, please check Network."
        return ${inferenceError}
      fi
    elif [[ ${onnx_model}"x" != "x" ]];then
      wget -O ${project_path}/model/${onnx_model##*/} ${onnx_model} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install onnx_model failed, please check Network."
        return ${inferenceError}
      fi
    elif [[ ${mindspore_model}"x" != "x" ]];then
      wget -O ${project_path}/model/${mindspore_model##*/} ${mindspore_model} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install onnx_model failed, please check Network."
        return ${inferenceError}
      fi
    elif [[ ${json_model}"x" != "x" ]];then
      wget -O ${project_path}/model/${json_model##*/} ${json_model} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "install json_model failed, please check Network."
        return ${inferenceError}
      fi
    else
      echo "No model download link available, please confirm"
      return ${inferenceError}
    fi
    
    if [[ ${aipp_cfg}"x" != "x" ]];then
      wget -O ${project_path}/model/${aipp_cfg##*/} ${aipp_cfg} --no-check-certificate
      if [ $? -ne 0 ];then
        echo "download aipp_cfg failed, please check Network."
        return ${inferenceError}
      fi
    fi

    return ${success}
}
function modelconvert()
{
  setEnv
  mkdir -p ${HOME}/models/${project_name}
  if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then
    # 下载原始模型文件[aipp_cfg文件]
    downloadOriginalModel
    if [ $? -ne 0 ];then
      return ${inferenceError}
    fi

    # 转模型
    cd ${project_path}/model/
    ${model_atc}
    if [ $? -ne 0 ];then
      echo "ERROR: convert model failed"
      return ${inferenceError}
    fi
  fi

  if [[ $(find ${project_path}/model -name ${model_name}".om")"x" != "x" ]];then
    rm ${project_path}/model/${model_name}".om"
  fi

  ln -s ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
  if [ $? -ne 0 ];then
    echo "ERROR: failed to set model soft connection"
    return ${inferenceError}
  fi
  return ${success}

}
function buildproject()
{
  setEnv
  Kernel=`uname -m`
  if [[ ${Kernel} = "x86_64" ]];then
    TargetKernel="x86"
    cxx_compiler="g++"
  else
    TargetKernel="arm"
    cxx_compiler="aarch64-linux-gnu-g++"
  fi

  echo "cxx_compiler=${cxx_compiler}"
  # 创建目录用于存放编译文件
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host


    # 产生Makefile
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=${cxx_compiler} -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "ERROR: cmake failed. please check your project"
        return ${inferenceError}
    fi

    make
    if [ $? -ne 0 ];then
        echo "ERROR: make failed. please check your project"
        return ${inferenceError}
    fi
    return ${success}
}
function run_picture()
{
    mkdir -p ${project_path}/out/output
    # 运行程序
    cd ${project_path}/out
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out/output" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out/output -name "*${tmp#*_}"`;do
            python3.6 ${common_script_dir}/verify_result.py ${test_file} ${outimage}
            if [ $? -ne 0 ];then
                echo "ERROR: The result of reasoning is wrong!"
                return ${verifyResError}
            fi
        done
    done

    echo "run success"

    return ${success}
}
function run_picture_python()
{
    # 运行程序
    cd ${project_path}/src
    ${run_command}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    # 调用python脚本判断本工程推理结果是否正常
    for outimage in $(find ${project_path}/verify_image -name "*.jpg");do
        tmp=`basename $outimage`
        if [[ ! -d "${project_path}/out" ]];then
            echo "ERROR: not find results folders!"
            return ${verifyResError}
        fi
        for test_file in `find ${project_path}/out -name "*${tmp#*_}"`;do
            python3.6 ${common_script_dir}/verify_result.py ${test_file} ${outimage}
            if [ $? -ne 0 ];then
                echo "ERROR: The result of reasoning is wrong!"
                return ${verifyResError}
            fi
        done
    done
    echo "run success"
    return ${success}
}
function run_presenter()
{
  # 开启presenter server
    cd ${common_script_dir}
    bash run_presenter_server.sh ${script_path}/${conf_file_name}
    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi

    sleep 2
    # 运行程序
    mv ${project_path}/out/main ${project_path}/out/${project_name}
    cd ${project_path}/out/
    ${run_command} &

    sleep 3

    project_pid=`ps -ef | grep "${project_name}"  | grep -v "grep" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

        presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep -v "grep" | awk -F ' ' '{print $2}'`
        if [[ ${presenter_server_pid}"X" != "X" ]];then
            echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
            kill -9 ${presenter_server_pid}
            if [ $? -ne 0 ];then
                echo "ERROR: kill presenter server process failed."
                return ${inferenceError}
            fi
        fi
    else
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    echo "run success"

    return ${success}  

}
function run_presenter_python()
{
  # 开启presenter server
    cd ${common_script_dir}
    bash run_presenter_server.sh ${script_path}/${conf_file_name}
    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi

    sleep 2
    # 运行程序
    cd ${project_path}/src
    ${run_command} &

    sleep 3

    project_pid=`ps -ef | grep "${run_command}"  | grep -v "grep" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

        presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep -v "grep" | awk -F ' ' '{print $2}'`
        if [[ ${presenter_server_pid}"X" != "X" ]];then
            echo -e "\033[33mNow do presenter server configuration, kill existing presenter process: kill -9 ${presenter_server_pid}.\033[0m"
            kill -9 ${presenter_server_pid}
            if [ $? -ne 0 ];then
                echo "ERROR: kill presenter server process failed."
                return ${inferenceError}
            fi
        fi
    else
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    echo "run success"

    return ${success}  
}
function run_md5()
{
    mkdir -p ${project_path}/out/output
    # 运行程序
    cd ${project_path}/out
    ${run_command}
    #if [ $? -ne 0 ];then
    #    echo "ERROR: run failed. please check your project"
    #    return ${inferenceError}
    #fi
    sleep 1

    output_result=$(ls ${project_path}/out/output/${verify_name} 2>/dev/null)
    verify_result=$(ls ${project_path}/verify_image/${verify_name} 2>/dev/null)
    if [[ ${output_result}"x" = "x" ]] || [[ ${verify_result}"x" = "x" ]];then
        echo "ERROR: verify failed. please check your project"
        return ${verifyResError}
    fi

    result_md5=`md5sum ${project_path}/out/output/${verify_name} | cut -d " " -f1`
    verify_md5=`md5sum ${project_path}/verify_image/${verify_name} | cut -d " " -f1`
    if [[ ${result_md5} != ${verify_md5} ]];then
      echo "ERROR: verify failed. please check your project"
      return ${verifyResError}
    fi

    echo "run success"

    return ${success}
}
function run_md5_python()
{
    mkdir -p ${project_path}/out
    # 运行程序
    cd ${project_path}/src
    ${run_command}

    sleep 1

    output_result=$(ls ${project_path}/out/${verify_name} 2>/dev/null)
    verify_result=$(ls ${project_path}/verify_image/${verify_name} 2>/dev/null)
    if [[ ${output_result}"x" = "x" ]] || [[ ${verify_result}"x" = "x" ]];then
        echo "ERROR: verify failed. please check your project"
        return ${verifyResError}
    fi

    result_md5=`md5sum ${project_path}/out/${verify_name} | cut -d " " -f1`
    verify_md5=`md5sum ${project_path}/verify_image/${verify_name} | cut -d " " -f1`
    if [[ ${result_md5} != ${verify_md5} ]];then
      echo "ERROR: verify failed. please check your project"
      return ${verifyResError}
    fi

    echo "run success"

    return ${success}
}

function run_txt()
{
    mkdir -p ${project_path}/out/output
    # 运行程序
    cd ${project_path}/out
    sleep 1

    ${run_command} > ${txt_file}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   

    result=`diff -q ${txt_file} ${project_path}/verify_image/${verify_name}`
    if [[ $result != "" ]]; then
        echo "Error: The result of reasoning is wrong, Both file are different!"
	  return ${verifyResError}
    fi

    echo "run success"

    return ${success}
}
