caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt"
model_name="yolov3_yuv"
aipp_cfg="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/aipp_nv12.cfg"
project_name="python_coco_detection_rtsp"

presenter_server_name="coco_detection"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_rtsp/person.h264"

project_name="python_coco_detection_rtsp"

version="c76"



script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    wget -O ${project_path}/data/"person.h264"  ${data_source}  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download person.h264 failed, please check Network."
        return 1
    fi

    return 0
}



function setAtcEnv() {
    if [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    elif [[ ${version} = "c76" ]] || [[ ${version} = "C76" ]];then
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
        export LD_LIBRARY_PATH=${install_path}/atc/lib64:${LD_LIBRARY_PATH}
    fi
    return 0
}

function downloadOriginalModel() {

    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${caffe_model##*/} ${caffe_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${aipp_cfg##*/}  ${aipp_cfg} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_model failed, please check Network."
        return 1
    fi

    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # 下载测试集和验证集
    downloadDataWithVerifySource
    if [ $? -ne 0 ];then
        echo "ERROR: download test images or verify images failed"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        # 下载原始模型文件[aipp_cfg文件]
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

        # 设置模型转换的环境变量
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

        cd ${project_path}/model/
        atc --model=${project_path}/model/${caffe_prototxt##*/} --weight=${project_path}/model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --save_original_model=false
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
    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}
    export PYTHONPATH=/home/HwHiAiUser/Ascend/nnrt/latest/pyACL/python/site-packages/acl:${PYTHONPATH}

    sleep 2  


    cd ${project_path}/src
    python3.6 main.py &
    
    sleep 8
    echo ${project_name}    
    project_pid=`ps -ef | grep "main.py" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

    else 
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    echo "run success"

    return ${success}
}
main
