face_detection_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel"
face_detection_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt"
face_detection_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_recognition_camera/face_detection_insert_op.cfg"

vanillacnn_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vanillacnn/vanillacnn.caffemodel"
vanillacnn_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/vanillacnn/vanilla_deploy.prototxt"

sphereface_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sphereface/sphereface.caffemodel"
sphereface_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sphereface/sphereface.prototxt"
sphereface_cfg="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_recognition_camera/sphereface_insert_op.cfg"

model_name1="face_detection"
model_name2="vanillacnn"
model_name3="sphereface"
presenter_server_name="facial_recognition"

declare -a modelArray

project_name="cplusplus_face_facial_recognition"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2



function setAtcEnv() {
    # 设置模型转换时需要的环境变量
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

function setBuildEnv() {
    # 设置代码编译时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}

function downloadOriginalModel() {
    
    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${face_detection_prototxt##*/} ${face_detection_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install face_detection_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${face_detection_model##*/} ${face_detection_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install face_detection_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${face_detection_cfg##*/}  ${face_detection_cfg} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install face_detection_cfg failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${vanillacnn_prototxt##*/} ${vanillacnn_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install vanillacnn_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${vanillacnn_model##*/} ${vanillacnn_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install vanillacnn_model failed, please check Network."
        return 1
    fi

    
    wget -O ${project_path}/model/${sphereface_prototxt##*/} ${sphereface_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install sphereface_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${sphereface_model##*/} ${sphereface_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install sphereface_model failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/model/${sphereface_cfg##*/}  ${sphereface_cfg} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install sphereface_cfg failed, please check Network."
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
    declare -i num=0
    for i in $(find ${HOME}/models/${project_name} -name "*.om" 2>/dev/null);do
        modelArray[$((num++))]=${i}
    done 
    if [[ 3 -eq ${#modelArray[@]} ]];then 
        ln -s ${HOME}/models/${project_name}/${model_name1}".om" ${project_path}/model/${model_name1}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name2}".om" ${project_path}/model/${model_name2}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name3}".om" ${project_path}/model/${model_name3}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

    else 
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

        # 转模型
        cd ${project_path}/model/
        atc --model=${project_path}/model/${face_detection_prototxt##*/} --weight=${project_path}/model/${face_detection_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name1} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${face_detection_cfg##*/} --input_shape="data:1,3,300,300" --input_format=NCHW
            if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        atc --model=${project_path}/model/${vanillacnn_prototxt##*/} --weight=${project_path}/model/${vanillacnn_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name2} --soc_version=Ascend310 --input_shape="data:4,3,40,40" --input_format=NCHW
            if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        atc --model=${project_path}/model/${sphereface_prototxt##*/} --weight=${project_path}/model/${sphereface_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${model_name3} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${sphereface_cfg##*/} --input_shape="data:8,3,112,96" --input_format=NCHW
            if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name1}".om" ${project_path}/model/${model_name1}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name2}".om" ${project_path}/model/${model_name2}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name3}".om" ${project_path}/model/${model_name3}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi

    fi

    # 创建目录用于存放编译文件
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    # 产生Makefile
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
    if [ $? -ne 0 ];then
        echo "ERROR: cmake failed. please check your project"
        return ${inferenceError}
    fi

    make 
    if [ $? -ne 0 ];then
        echo "ERROR: make failed. please check your project"
        return ${inferenceError}
    fi

    

    cd ${project_path}/out

    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:${LD_LIBRARY_PATH}
    
    # 开启presenter server
    echo "./
    
    " | bash ${script_path}/run_presenter_server.sh 
    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi

    sleep 2

    # 运行程序
    mv main ${project_name}
    ./${project_name} &
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi   
    
    sleep 8

    project_pid=`ps -ef | grep "${project_name}" | awk -F ' ' '{print $2}'`
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