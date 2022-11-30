#!/bin/bash
tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/Hand_detection.pb"
test_img="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/hand.jpg"
verify_img="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/verify.jpg"
model_name="Hand_detection"
version=$1
data_source="../data/"
verify_source="../data/"
project_name="Hand_detection"
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function setAtcEnv() {

    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export PYTHONPATH=${install_path}/atc/python/site-packages/te:${install_path}/atc/python/site-packages/topi:$PYTHONPATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    elif [[ ${version} = "c78" ]] || [[ ${version} = "C78" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export PYTHONPATH=${install_path}/pyACL/python/site-packages/acl:$PYTHONPATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export ASCEND_AICPU_PATH=$HOME/Ascend/ascend-toolkit/latest
        export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:$LD_LIBRARY_PATH
        export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
    fi

    return 0
}

function setRunEnv() {

    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl:${PYTHONPATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux/pyACL/python/site-packages/acl:${PYTHONPATH}
    elif [[ ${version} = "c78" ]] || [[ ${version} = "C75" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux/pyACL/python/site-packages/acl:${PYTHONPATH}
    fi

    return 0
}


function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    echo ${tf_model}

    wget ${tf_model} -O ${project_path}/model/${model_name}.pb --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tf_model failed, please check Network."
        return 1
    fi


    return 0
}

function downloadTestData() {

    echo ${test_img}
    echo ${verify_img}

    wget ${test_img} --no-check-certificate -O ${project_path}/data/hand.jpg
    wget ${verify_img} --no-check-certificate -O ${project_path}/data/verify.jpg
    if [ $? -ne 0 ];then
        echo "Download test images failed, please check Network."
        return 1
    fi
    return 0
}


function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 

        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

        downloadTestData
        if [ $? -ne 0 ];then
            echo "ERROR: download test data failed"
            return ${inferenceError}
        fi

        # setAtcEnv
        export LD_LIBRARY_PATH=${install_path}/compiler/lib64:$LD_LIBRARY_PATH
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        cd ${project_path}/model/
        atc --model=${project_path}/model/Hand_detection.pb --framework=3 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --input_shape="image_tensor:1,300,300,3" --input_format=NHWC --output_type=FP32
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -s ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    fi

    cd ${project_path}


    # setRunEnv
    source ~/.bashrc
    if [ $? -ne 0 ];then
        echo "ERROR: set executable program running environment failed"
        return ${inferenceError}
    fi

    #test:hand.jpg
    original_img=${project_path}/data/hand.jpg
    verify_img=${project_path}/data/verify.jpg
    mkdir -p ${project_path}/output
    rm ${project_path}/output/*
    python3.6 ${project_path}/src/hand_detection.py --input_image ${original_img}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    out_img=${project_path}/output/output.jpg
    python3 ${script_path}/verify_result.py ${verify_img} ${out_img}
    if [ $? -ne 0 ];then
        echo "ERROR: The result of test 1 is wrong!"
        return ${verifyResError}
    fi   

    echo "********run test success********"

    return ${success}
}
main
