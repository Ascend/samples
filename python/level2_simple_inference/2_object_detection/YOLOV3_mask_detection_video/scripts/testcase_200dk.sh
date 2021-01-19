tensorflow_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/yolo3_resnet18.pb"
aipp_cfg="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/insert_op.cfg"

model_name="yolo3_resnet18_yuv"
presenter_server_name="mask_detection_video"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mask_detection_video/mask-true.mp4"
project_name="YOLOV3_mask_detection_video"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2



function downloadData() {

    mkdir -p ${project_path}/data/

    wget -O ${project_path}/data/"mask-true.mp4"  ${data_source}  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download ture.mp4 failed, please check Network."
        return 1
    fi

    return 0
}


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
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl:${PYTHONPATH}
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export LD_LIBRARY_PATH=
        export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
        export PYTHONPATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux/pyACL/python/site-packages/acl:${PYTHONPATH}
    fi

    return 0
}

function downloadOriginalModel() {
    
    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${tensorflow_model##*/} ${tensorflow_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install tensorflow_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${aipp_cfg##*/} ${aipp_cfg} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install aipp_cfg failed, please check Network."
        return 1
    fi
    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

        
    downloadData
    if [ $? -ne 0 ];then
        echo "ERROR: download test images failed"
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}     
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name}".om")"x" = "x" ]];then 
        
        downloadOriginalModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original model failed"
            return ${inferenceError}
        fi

              
        setAtcEnv
        if [ $? -ne 0 ];then
            echo "ERROR: set atc environment failed"
            return ${inferenceError}
        fi

                
        cd ${project_path}/model/
        
        atc --model=${project_path}/model/${tensorflow_model##*/} --insert_op_conf=${project_path}/model/${aipp_cfg##*/} --output=${HOME}/models/${project_name}/${model_name}  --output_type=FP32 --input_shape="images:1,352,640,3" --input_format=NHWC --framework=3 --soc_version=Ascend310



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

    


    setRunEnv
    
     if [ $? -ne 0 ];then
        echo "ERROR: set executable program running environment failed"
        return ${inferenceError}
    fi

    echo "127.0.0.1
    
    " | bash ${script_path}/run_presenter_server.sh 
    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi
    

    sleep 2
    
    cd ${project_path}/src
    #python3.6 mask_detect.py
    python3.6 main.py ../data/mask-true.mp4 &
    
    sleep 8
    echo ${project_name}    
        project_pid=`ps -ef | grep "main.py" | grep "data" | awk -F ' ' '{print $2}'`
    if [[ ${project_pid}"X" != "X" ]];then
        echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
        kill -9 ${project_pid}
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

        presenter_server_pid=`ps -ef | grep "presenter_server\.py" | grep "${presenter_server_name}" | awk -F ' ' '{print $2}'`
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
main
