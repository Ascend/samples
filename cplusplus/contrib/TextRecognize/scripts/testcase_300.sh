crnn_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/crnn_static/crnn_static.pb"
dbnet_model="https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/dbnet/dbnet.pb"
data_source="https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/TextRecognize/"

dbnet_model_name="dbnet"
crnn_model_name="crnn_static"
presenter_server_name="text_recognize"
project_name="TextRecognize"

version=$1

script_path="$( cd "$(dirname $BASH_SOURCE)" ; pwd -P)"
project_path=${script_path}/..

declare -i success=0
declare -i inferenceError=1
declare -i verifyResError=2

function downloadData() {
    mkdir -p ${project_path}/data/
    wget -O ${project_path}/data/"data.mp4"  ${data_source}"data.mp4"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download data.mp4 failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"char_dict_en.json"  ${data_source}"char_dict_en.json"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download char_dict_en.json failed, please check Network."
        return 1
    fi

    wget -O ${project_path}/data/"ord_map_en.json"  ${data_source}"ord_map_en.json"  --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download ord_map_en.json failed, please check Network."
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

function setAtcEnv() {
    # set enviroment param
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
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    elif [[ ${version} = "c75" ]] || [[ ${version} = "C75" ]];then
        export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=${DDK_PATH}/acllib/lib64/stub
    fi
    return 0
}

function downloadOriginalDbnetModel() {
    wget -O ${project_path}/model/${dbnet_model##*/} ${dbnet_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download dbnet_model failed, please check Network."
        return 1
    fi
    return 0
}

function downloadOriginalCrnnModel() {
    wget -O ${project_path}/model/${crnn_model##*/} ${crnn_model} --no-check-certificate
    if [ $? -ne 0 ];then
        echo "download crnn_model failed, please check Network."
        return 1
    fi
    return 0
}

function main() {

    if [[ ${version}"x" = "x" ]];then
        echo "ERROR: version is invalid"
        return ${inferenceError}
    fi

    # 下载测试集
    downloadData
    if [ $? -ne 0 ];then
        echo "ERROR: download test images failed"
        return ${inferenceError}
    fi
    
    mkdir -p ${HOME}/models/${project_name}
    mkdir -p ${project_path}/model/   
    if [[ $(find ${HOME}/models/${project_name} -name ${dbnet_model_name}".om")"x" = "x" ]];then 
        downloadOriginalDbnetModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original dbnet model failed"
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
        atc --model=${project_path}/model/${dbnet_model##*/}  --framework=3 --output=${HOME}/models/${project_name}/${dbnet_model_name} --soc_version=Ascend310  --output_type=FP32 --input_shape="input_images:1,736,1312,3" --input_format=NHWC
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${dbnet_model_name}".om" ${project_path}/model/${dbnet_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -s ${HOME}/models/${project_name}/${dbnet_model_name}".om" ${project_path}/model/${dbnet_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    fi

    if [[ $(find ${HOME}/models/${project_name} -name ${crnn_model_name}".om")"x" = "x" ]];then 
        downloadOriginalCrnnModel
        if [ $? -ne 0 ];then
            echo "ERROR: download original dbnet model failed"
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
        atc --model=${project_path}/model/${crnn_model##*/}  --framework=3 --output=${HOME}/models/${project_name}/${crnn_model_name} --soc_version=Ascend310  --input_shape="new_input:1,32,100,3" --input_format=NHWC
        if [ $? -ne 0 ];then
            echo "ERROR: convert model failed"
            return ${inferenceError}
        fi

        ln -s ${HOME}/models/${project_name}/${crnn_model_name}".om" ${project_path}/model/${crnn_model_name}".om"
        if [ $? -ne 0 ];then
            echo "ERROR: failed to set model soft connection"
            return ${inferenceError}
        fi
    else 
        ln -s ${HOME}/models/${project_name}/${crnn_model_name}".om" ${project_path}/model/${crnn_model_name}".om"
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

    # 重新配置程序运行所需的环境变量
    export LD_LIBRARY_PATH=
    export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/nnrt/latest/acllib/lib64:/home/HwHiAiUser/ascend_ddk/x86/lib:$LD_LIBRARY_PATH

    # presenter server
    cd ${project_path}/../../../common/
    bash run_presenter_server.sh ${script_path}/TextRecongnize.conf &

    if [ $? -ne 0 ];then
        echo "ERROR: run presenter server failed. please check your project"
        return ${inferenceError}
    fi

    sleep 5
    # run program
    cd ${project_path}/out
    mv ${project_path}/out/main ${project_path}/out/textrecognize

    ./textrecognize &

    sleep 5

    project_pid=`ps -ef | grep "textrecognize" | awk -F ' ' '{print $2}'`
        if [[ ${project_pid}"X" != "X" ]];then
            echo -e "\033[33m kill existing project process: kill -9 ${project_pid}.\033[0m"
            kill -9 ${project_pid}    
        if [ $? -ne 0 ];then
            echo "ERROR: kill project process failed."
            return ${inferenceError}
        fi

        presenter_server_pid=`ps -ef | grep "presenter_server\.py" | awk -F ' ' '{print $2}'`
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