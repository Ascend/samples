#!/bin/bash
tf_model="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/OpenPose_light.pb"
# om_model https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/OpenPose_light.om
images_link="https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture"
version=$1
model_name="OpenPose_light"
project_name="body_pose_picture"
script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function setAtcEnv() {

    if [[ ${version} = "c78" ]] || [[ ${version} = "C78" ]];then
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
    fi

    return 0
}

function setRunEnv() {

    if [[ ${version} = "c78" ]] || [[ ${version} = "C78" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl:${PYTHONPATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux/pyACL/python/site-packages/acl:${PYTHONPATH}
    fi

    return 0
}


function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    echo ${tf_model}

    wget ${tf_model} -O ${project_path}/model/${model_name}.pb
    if [ $? -ne 0 ];then
        echo "install tf_model failed, please check Network."
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
    # Model conversion
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 

        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

        # setAtcEnv
        export LD_LIBRARY_PATH=${install_path}/compiler/lib64:$LD_LIBRARY_PATH
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        cd ${project_path}/model/
        atc --framework=3 --model=${project_path}/model/OpenPose_light.pb --input_shape="input001:1,368,368,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${model_name} --output_type=FP32 --out_nodes="light_openpose/stage_1/ArgMax:0" --soc_version=Ascend310
        if [ $? -ne 0 ];then
            echo "ERROR: convert YOLO model failed"
            return ${inferenceError}
        fi

        ln -sf ${HOME}/models/${project_name}/${yolo_model_name}".om" ${project_path}/model/${yolo_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set YOLO model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -sf ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
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

    test_img=${project_path}/data/test.jpg
    verify_img=${project_path}/data/verify.jpg
    wget --no-check-certificate ${images_link}/test.jpg -O ${test_img}
    wget --no-check-certificate ${images_link}/verify.jpg -O ${verify_img}

    cd ${project_path}/src/
    python3 run_image.py --frames_input_src ${test_img}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    out_img=${project_path}/output/test_output.jpg
    python3 ${script_path}/verify_result.py ${verify_img} ${out_img}
    if [ $? -ne 0 ];then
        echo "ERROR: The result of test 1 is wrong!"
        return ${verifyResError}
    fi   

    echo "********run test success********"

    return ${success}
}
main
