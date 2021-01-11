caffe_model="https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/yolov3/yolov3.caffemodel"
caffe_prototxt="https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/yolov3/yolov3.prototxt"
project_name="cplusplus_YOLOV3_dynamic_batch_detection_picture"
batch_model_name="yolov3_dynamic_batch"
hw_model_name="yolov3_dynamic_hw"
version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function generateData() {
    # 生成模型输入文件
    cd ${project_path}/data/
    python3 tools_generate_data.py input -s [1,3,416,416] -r [2,3] -d float32
    if [ $? -ne 0 ];then
        echo "generate data failed."
        return 1
    fi

    return 0
}

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

function downloadOriginalModel() {
    mkdir -p ${project_path}/caffe_model/
    
    wget -O ${project_path}/caffe_model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_prototxt failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/caffe_model/${caffe_model##*/} ${caffe_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "install caffe_model failed, please check Network."
        return 1
    fi

    return 0
}

function setBuildEnv() {
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}

function main() {
    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # 生成模型输入数据集
    generateData
    if [ $? -ne 0 ];then
        echo "ERROR: generate data failed."
        return ${inferenceError}
    fi

    mkdir -p ${HOME}/models/${project_name}
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir models folder failed. please check your project"
        return ${inferenceError}
    fi
    mkdir -p ${project_path}/model/
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir model folder failed. please check your project"
        return ${inferenceError}
    fi
    if [[ $(find ${HOME}/models/${project_name} -name ${batch_model_name}".om")"x" = "x" ]] || [[ $(find ${HOME}/models/${project_name} -name ${hw_model_name}".om")"x" = "x" ]];then 
        # 下载原始模型文件
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
        atc --model=${project_path}/caffe_model/${caffe_prototxt##*/} --weight=${project_path}/caffe_model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${batch_model_name} --input_shape="data:-1,3,416,416"  --input_format=NCHW --dynamic_batch_size="1,2,4,8" --soc_version=Ascend310
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        atc --model=${project_path}/caffe_model/${caffe_prototxt##*/} --weight=${project_path}/caffe_model/${caffe_model##*/} --framework=0 --output=${HOME}/models/${project_name}/${hw_model_name} --input_shape="data:1,3,-1,-1"  --input_format=NCHW --dynamic_image_size="416,416;832,832;1248,1248" --soc_version=Ascend310
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${batch_model_name}".om" ${project_path}/model/${batch_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
        ln -s ${HOME}/models/${project_name}/${hw_model_name}".om" ${project_path}/model/${hw_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -s ${HOME}/models/${project_name}/${batch_model_name}".om" ${project_path}/model/${batch_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
        ln -s ${HOME}/models/${project_name}/${hw_model_name}".om" ${project_path}/model/${hw_model_name}".om"
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

    # 设置代码编译时需要的环境变量
    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    # 产生Makefile
    cmake ${project_path}/src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
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
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:$LD_LIBRARY_PATH

    # 运行程序
    ./main 1 > result.txt
    if [ $? -ne 0 ];then
        echo "ERROR: run dynamic batch failed. please check your project"
        return ${inferenceError}
    fi

    # 验证结果
    results=$(cat result.txt)
    if [[ $results =~ "ERROR" || $results =~ "fail" || $results =~ "FAIL" || $results =~ "error" ]];then
        echo "run failed"
        return ${verifyResError}
    else
        echo "run success"
        return ${success}
    fi

    # 运行程序
    ./main 416 416 > result.txt
    if [ $? -ne 0 ];then
        echo "ERROR: run dynamic hw failed. please check your project"
        return ${inferenceError}
    fi

    # 验证结果
    results=$(cat result.txt)
    if [[ $results =~ "ERROR" || $results =~ "fail" || $results =~ "FAIL" || $results =~ "error" ]];then
        echo "run failed"
        return ${verifyResError}
    else
        echo "run success"
        return ${success}
    fi
}
main
