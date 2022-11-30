#!/bin/bash
om_model='https://docs.google.com/uc?export=download&id=1RU1UBVH5EBbVV4CVAPuNokSzpfx9A3Ug'
onnx_model='https://docs.google.com/uc?export=download&id=1Esjf7Mj-CTh-VQGNHEcpX2uwJlQEnrJD'
onnx_model2='https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/mot_v2.onnx'
model_name="mot_v2"
version=$1
test_img_link='https://docs.google.com/uc?export=download&id=1WWNPkZ9jIfbufZ8SzpUqQiuPTHo_h-lA'
test_img_link2='https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/test.jpg'
verify_img_link='https://docs.google.com/uc?export=download&id=1g9XOWpkZzCZxtUUCj4PvzAgjesBs-jJJ'
verify_img_link2='https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/verify.jpg'
project_name="object_tracking_video"
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
    fi

    return 0
}

function downloadOriginalModel() {
    mkdir -p ${project_path}/model/
    wget --no-check-certificate ${onnx_model} -O ${project_path}/model/${model_name}.onnx

    if [ $? -ne 0 ];then

        echo "Download mot_v2.onnx failed from Google Drive, Retrying from HuaweiCloud."
        wget --no-check-certificate ${onnx_model2} -O ${project_path}/model/${model_name}.onnx

        if [ $? -ne 0 ];then
            echo "Download mot_v2.onnx failed, please check Network."
            return 1
        fi
    fi

    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    # mot_v2 conversion (-if find returns an empty string, then download model)
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 

        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original mot_v2 model failed"
            return ${inferenceError}
        fi

        # setAtcEnv
        export LD_LIBRARY_PATH=${install_path}/compiler/lib64:$LD_LIBRARY_PATH
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        # Is this step necessary if we downloaded a .om?
        cd ${project_path}/model/
        atc --input_shape="input.1:1,3,608,1088" --check_report=./network_analysis.report --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --framework=5 --model=${project_path}/model/${model_name}.onnx 
        # atc --framework=3 --model=${project_path}/model/yolo_model.pb --input_shape="input_1:1,416,416,3" --input_format=NHWC --output=${HOME}/models/${project_name}/${yolo_model_name} --output_type=FP32 --soc_version=Ascend310
        if [ $? -ne 0 ];then
            echo "ERROR: convert mot_v2 model failed"
            return ${inferenceError}
        fi

        ln -sf ${HOME}/models/${project_name}/${model_name}".om" ${project_path}/model/${model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set mot_v2 model soft connection"
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
    

    # Downlaod test images
    mkdir -p ${project_path}/data/
    wget -O ${project_path}/data/"test.jpg"  ${test_img_link} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download test.jpg failed from Google Drive, Retrying from HuaweiCloud."
        wget -O ${project_path}/data/"test.jpg"  ${test_img_link2} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "Download test.jpg failed, please check Network."
            return 1
        fi
    fi
    wget -O ${project_path}/data/"verify.jpg"  ${verify_img_link} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download verify.jpg failed from Google Drive, Retrying from HuaweiCloud."
        wget -O ${project_path}/data/"verify.jpg"  ${verify_img_link} --no-check-certificate
        if [ $? -ne 0 ];then
            echo "Download verify.jpg failed, please check Network."
            return 1
        fi
    fi

    #*** change paths to images ***
    test_img=${project_path}/data/test.jpg
    verify_img=${project_path}/data/verify.jpg
    cd  ${project_path}/src
    python3 test.py --test_img ${test_img} --verify_img  ${verify_img}
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   

    echo "********run test success********"

    return ${success}
}
main

#1. get/change path to test images 
#2. check if we can just download an .om model without using atc to convert
#3. test the script
