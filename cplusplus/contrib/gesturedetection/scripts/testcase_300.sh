caffe_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_iter_440000.caffemodel"
caffe_prototxt="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_deploy.prototxt"
caffe_insert_op="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/insert_op.cfg"
tensorflow_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/stgcn_fps30_sta_ho_ki4.pb"
model_name_1="pose_deploy"
model_name_2="stgcn_fps30_sta_ho_ki4"

data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/gesturedetection/test_image/"
project_name="cplusplus_Gesturedetection"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2


function downloadDataWithVerifySource() {

    mkdir -p ${project_path}/data/

    for ((i=0; i<50; i ++))
    do
    wget -O ${project_path}/data/$i".jpg"  ${data_source}$i".jpg"  --no-check-certificate
    if [ $? -ne 0 ];then
            echo "Download jpg failed, please check network."
            return 1
        fi
    done

    return 0
}


function setAtcEnv() {
    # 设置模型转换时需要的环境变量
    if [[ ${version} = "c73" ]] || [[ ${version} = "C73" ]];then
        export install_path=$HOME/Ascend/ascend-toolkit/latest
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
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi

    return 0
}

function downloadOriginalModel() {
    
    mkdir -p ${project_path}/model/

    wget -O ${project_path}/model/${caffe_prototxt##*/} ${caffe_prototxt} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download caffe_prototxt failed, please check network."
        return 1
    fi

    wget -O ${project_path}/model/${caffe_model##*/} ${caffe_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download caffe_model failed, please check network."
        return 1
    fi

    wget -O ${project_path}/model/${caffe_insert_op##*/} ${caffe_insert_op} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download caffe_insert_op failed, please check network."
        return 1
    fi

    wget -O ${project_path}/model/${tensorflow_model##*/} ${tensorflow_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "Download tensorflow_model failed, please check network."
        return 1
    fi

    return 0
}

function buildLibAtlasUtil() {
    cd ${project_path}/../../common/atlasutil/
    make mode=ASIC
    if [ $? -ne 0 ];then
        echo "ERROR: make atlasutil failed."
        return ${inferenceError}
    fi

    make mode=ASIC install
    if [ $? -ne 0 ];then
        echo "ERROR: make install atlasutil failed."
        return ${inferenceError}
    fi
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
    if [[ $(find ${HOME}/models/${project_name} -name ${model_name_1}".om")"x" = "x" ]] || [[ $(find ${HOME}/models/${project_name} -name ${model_name_2}".om")"x" = "x" ]];then 
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
        atc --input_shape="data:1,3,128,128" --weight=${project_path}/model/${caffe_model##*/} --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name_1} --soc_version=Ascend310 --insert_op_conf=${project_path}/model/${caffe_insert_op##*/} --framework=0 --model=${project_path}/model/${caffe_prototxt##*/}
        if [ $? -ne 0 ];then
            echo "ERROR: convert model_1 failed"
            return ${inferenceError}
        fi

        atc --input_shape="input_features:1,2,30,14" --input_format=NCHW --output=${HOME}/models/${project_name}/${model_name_2} --soc_version=Ascend310 --framework=3 --model=${project_path}/model/${tensorflow_model##*/}
        if [ $? -ne 0 ];then
            echo "ERROR: convert model_2 failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name_1}".om" ${project_path}/model/${model_name_1}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model_1 soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name_2}".om" ${project_path}/model/${model_name_2}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model_2 soft connection"
            return ${inferenceError}
        fi

    else
        ln -s ${HOME}/models/${project_name}/${model_name_1}".om" ${project_path}/model/${model_name_1}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model_1 soft connection"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${model_name_2}".om" ${project_path}/model/${model_name_2}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model_2 soft connection"
            return ${inferenceError}
        fi
    fi

    setBuildEnv
    if [ $? -ne 0 ];then
        echo "ERROR: set build environment failed"
        return ${inferenceError}
    fi

    buildLibAtlasUtil
    if [ $? -ne 0 ];then
        echo "ERROR: build libatlasutil.so failed"
        return ${inferenceError}
    fi

    # 创建目录用于存放编译文件
    mkdir -p ${project_path}/build/intermediates/host
    if [ $? -ne 0 ];then
        echo "ERROR: mkdir build folder failed. please check your project"
        return ${inferenceError}
    fi
    cd ${project_path}/build/intermediates/host

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
    mkdir output
    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64:$HOME/ascend_ddk/x86/lib:${LD_LIBRARY_PATH}

    # 运行程序
    ./main
    if [ $? -ne 0 ];then
        echo "ERROR: run failed. please check your project"
        return ${inferenceError}
    fi

    echo "run success"

    return ${success}
}
main


